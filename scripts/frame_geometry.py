"""Deterministic, source-anchored native weldment recipes. SI coordinates throughout.

The recipe is a proposed frame family; feasibility is established by independent
native geometry/rule/study gates, not by successful recipe generation.
"""
from pathlib import Path
import json, math, argparse, hashlib
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'handoff/evidence/engineering-2026-10-11T14-18-45-377Z/stdout.json'
PROFILES={'A':'1.000 x 0.095.SLDLFP','B':'1.000 x 0.065.SLDLFP','C':'1.000 x 0.049.SLDLFP'}
def load(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def save(p,x):Path(p).parent.mkdir(parents=True,exist_ok=True);Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n',encoding='utf-8')

def source_geometry():
    raw=load(SOURCE)['result'];out={}
    for f in raw['features']:
        if not f.get('sketch'):continue
        s=f['sketch'];m=s['modelToSketch'];r=np.array(m[:9]).reshape(3,3)
        def world(p):return ((np.array(p)-m[9:12])@np.linalg.inv(r)).tolist()
        out[f['name']]=[{'kind':q['kind'],'construction':q['construction'],**{k:world(v) if k in ('start','end','center') else v for k,v in (q.get('detail') or {}).items()}} for q in s['segments']]
    return out,raw['bodies']

def hoop_at_z(segments,z,side):
    candidates=[]
    for s in segments:
        a=np.array(s['start']);b=np.array(s['end']);kind=s['kind']
        if kind in (0,'line'):
            if abs(b[2]-a[2])<1e-10:continue
            t=(z-a[2])/(b[2]-a[2])
            if -1e-8<=t<=1+1e-8:candidates.append(a+t*(b-a))
        else:
            c=np.array(s['center']);u=a-c;v=b-c;angle=math.acos(np.clip(np.dot(u,v)/np.linalg.norm(u)/np.linalg.norm(v),-1,1));n=np.cross(u,v);n/=np.linalg.norm(n);w=np.cross(n,u)
            def p(t):return c+u*math.cos(t)+w*math.sin(t)
            if (p(0)[2]-z)*(p(angle)[2]-z)>0:continue
            lo,hi=0,angle
            for _ in range(60):
                mid=(lo+hi)/2
                if (p(lo)[2]-z)*(p(mid)[2]-z)<=0:hi=mid
                else:lo=mid
            candidates.append(p((lo+hi)/2))
    pts=[x for x in candidates if x[1]*side>0]
    if not pts:raise ValueError(f'No hoop intersection at {z}')
    return max(pts,key=lambda x:abs(x[1])).tolist()

HOOP_OUTER_RADIUS=.0127  # supplied 1.000 x .095 Size A hoops
def outer_radius(member):
    """Outer radius (m) of a member's round profile; hoops when no profile is given."""
    import re
    if not member.get('profile'):return HOOP_OUTER_RADIUS
    m=re.match(r'(\d+\.\d+)\s*(SQ)?\s*x',member['profile'],re.I)
    if not m:raise ValueError('Unrecognised profile '+member['profile'])
    return float(m[1])*.0254/2

# Modelled cope fit-up gap. The procedure allows at most 0.05 in and prefers under 0.01 in.
# SOLIDWORKS Simulation also needs it small: its automatic beam joints extend a coped end to
# the joint only if the end "touches" its carrier, which on this install means a gap below
# about 0.5% of the smaller tube's OD (0.750 in joins at 0.08 mm, not 0.10 mm; 1.000 in at
# 0.12 mm, not 0.15 mm). A non-touching end keeps its raw beam end 8-28 mm short of the
# joint, beyond the procedure's 0.05 in clearance, and the tube is left unjoined.
# Evidence: handoff/evidence/joint-diagnostics-20261012/summary.json.
COPE_GAP_M=.0001
SMALL_TUBE_COPE_GAP_M=.00005  # any cope involving a tube under 1.000 in OD

def cope_gap(target,carrier):
    return SMALL_TUBE_COPE_GAP_M if min(outer_radius(target),outer_radius(carrier))<.0127-1e-9 else COPE_GAP_M

def miter_side_first(m,ts,carriers,group_of,nodes):
    """At a mitered kink (two members of one group meeting at a node), a tube whose cope
    footprint lies wholly past the miter, on one member's side, is coped onto that member
    first. The native cope then skips the other member (no remaining overlap) instead of
    shaping the end to the other member's extended cylinder. Coping onto the off-side
    member first left 009b's XBrace2 ends reported 1.46 mm from FrontLower*_1, a tube
    they never land on."""
    def unit(v):v=np.asarray(v,float);return v/np.linalg.norm(v)
    def outward(member,n):
        p=np.array(nodes[n]);a,b=np.array(member['a']),np.array(member['b'])
        return unit(b-a) if np.linalg.norm(a-p)<1e-6 else unit(a-b) if np.linalg.norm(b-p)<1e-6 else None
    def footprint_start(n,carrier):
        # Nearest edge of the target's footprint on the carrier, measured along the carrier
        # from the node; positive means it starts past the node on this carrier's side.
        u,v=outward(m,n),outward(carrier,n)
        if u is None or v is None:return None
        c=float(np.clip(u@v,-1,1));s=math.sqrt(1-c*c)
        return None if s<1e-6 else outer_radius(carrier)*c/s-outer_radius(m)/s
    ts=list(ts)
    for i in range(len(ts)):
        for j in range(i+1,len(ts)):
            a,b=ts[i],ts[j]
            if a['point']!=b['point'] or a['carrier'] not in carriers or b['carrier'] not in carriers:continue
            ca,cb=carriers[a['carrier']],carriers[b['carrier']]
            if group_of.get(ca['name'])!=group_of.get(cb['name']):continue
            n=next((k for k in set(ca['nodes'])&set(cb['nodes'])&set(m['nodes']) if nodes[k]==a['point']),None)
            if n is None:continue
            fb=footprint_start(n,cb)
            if fb is not None and fb>0:ts[i],ts[j]=b,a
    return ts

def make_trims(groups,nodes,hoop_nodes):
    """Cope list in weld order: each member is coped onto members/hoops placed before it."""
    # Proposed end contacts. Native audit also checks all unlisted intersections.
    trims=[];earlier=[{'name':name,'hoop':True,'nodes':ns} for name,ns in hoop_nodes.items()]
    group_of={mm['name']:g['name'] for g in groups for mm in g['members']}
    xhalf=lambda n:n.rsplit('_',1)[0] if n.startswith('XBrace') and n[-2:] in ('_A','_B') else None
    for g in groups:
        for m in g['members']:
            own=[]
            for carrier in earlier:
                # X-brace halves are coped onto the through tube only; their ends sit on
                # opposite sides of it and never touch each other.
                if xhalf(m['name']) and xhalf(m['name'])==xhalf(carrier['name']):continue
                shared=set(m['nodes'])&set(carrier['nodes'])
                # Also capture endpoint-to-interior and small source-offset joints.
                # These are candidates only: native intersection/contact checks govern.
                if not shared and not carrier.get('hoop'):
                    ca=np.array(carrier['a']); cb=np.array(carrier['b']); v=cb-ca
                    near=[]
                    for n in m['nodes']:
                        q=np.array(nodes[n]); t=np.clip(np.dot(q-ca,v)/np.dot(v,v),0,1)
                        if np.linalg.norm(q-(ca+t*v))<=outer_radius(m)+outer_radius(carrier)+1e-6:near.append(n)  # tube surfaces touch
                    shared=set(near)
                if not shared and carrier.get('hoop') and carrier['name']=='FrontHoop_1':
                    shared={n for n in m['nodes'] if n.startswith('FA')}
                if shared:
                    # One cope per touching end. A crossbar between both legs of the
                    # same hoop meets that single body twice; the first cut may leave
                    # the other end's overlap, which the final end's cut must clear.
                    ends=sorted(shared) if carrier.get('hoop') else sorted(shared)[:1]
                    for k,n in enumerate(ends):
                        t={'name':'Cope_'+m['name']+'_to_'+carrier['name']+('_'+n if k else ''),'target':m['name'],'carrier':carrier['name'],'gapM':cope_gap(m,carrier),'point':nodes[n]}
                        if k<len(ends)-1:t['partialEnd']=True
                        own.append(t)
            trims.extend(miter_side_first(m,own,{c['name']:c for c in earlier if not c.get('hoop')},group_of,nodes))
        earlier.extend(g['members'])
    return trims

BOUNDS={'cockpitHalfWidthM':(.295,.335),'cockpitStationXM':(.91,1.0),
        'sideUpperZM':(.350,.390),'frontSideUpperZM':(.3455,.374),'mainBraceZM':(1.246,1.285),'braceRearXM':(0,.10),
        'braceRearYM':(.26,.33),'braceRearZM':(.34,.42),'batteryPostXM':(.18,.24)}

def recipe(parameters=None,main_hoop='source',limits=None):
    """main_hoop: 'source' (the team's Main Roll Hoop Sketch, required from 009d on) or 'repair-002'
    (the 3.25 in die repair used up to 009c). limits: {parameter: (low, high)} overriding BOUNDS,
    e.g. a wider brace-rear window for a wider accumulator; recorded in the recipe."""
    import beam_model
    p={'cockpitHalfWidthM':.305,'cockpitStationXM':.945,'sideUpperZM':.360,'frontSideUpperZM':.3465,'mainBraceZM':1.247,
       'braceRearXM':.045,'braceRearYM':.280,'braceRearZM':.380,'batteryPostXM':.240,
       'rearBayProfile':'B','floorDiagonal':True}
    if parameters:
        unknown=set(parameters)-set(p)
        if unknown:raise ValueError(f'Unknown frame parameters: {unknown}')
        p.update(parameters)
    bounds={**BOUNDS,**(limits or {})}
    for key,(low,high) in bounds.items():
        if isinstance(p[key],bool) or not isinstance(p[key],(int,float)) or not math.isfinite(p[key]) or not low<=p[key]<=high:
            raise ValueError(f'{key} outside proposed family [{low}, {high}]')
    if p['rearBayProfile'] not in ('B','C') or not isinstance(p['floorDiagonal'],bool):raise ValueError('Unsupported stock or topology choice')
    src,refs=source_geometry();main=beam_model.main_hoop_segments(main_hoop);front=load(ROOT/'design/constraint-ledger.json')['hoops'][1]['worldSegments']
    def hoop_foot(segments,side):  # lowest end point of the hoop on that side
        return min((s[k] for s in segments for k in ('start','end') if s[k][1]*side>0),key=lambda q:q[2])
    nodes={};groups=[];members=[];hoop_nodes={'MainHoop_1':[],'FrontHoop_1':[]}
    def node(name,point,hoop=None):
        nodes[name]=[float(x) for x in point]
        if hoop:hoop_nodes[hoop].append(name)
        return name
    def group(name,role,size,pairs):
        ms=[]
        for i,(a,b) in enumerate(pairs):
            m={'name':name if len(pairs)==1 else name+'_'+str(i+1),'a':nodes[a],'b':nodes[b],'nodes':[a,b],'role':role,'size':size,'profile':PROFILES[size],'order':len(members)}
            ms.append(m);members.append(m)
        groups.append({'name':'Weld_'+name,'profile':PROFILES[size],'members':ms,'miter':len(pairs)>1})
        return ms
    for side,label in ((1,'L'),(-1,'R')):
        node('MH0'+label,hoop_foot(main,side),'MainHoop_1')
        node('MHU'+label,hoop_at_z(main,p['sideUpperZM'],side),'MainHoop_1')
        node('MHB'+label,hoop_at_z(main,p['mainBraceZM'],side),'MainHoop_1')
        node('MHH'+label,hoop_at_z(main,.640,side),'MainHoop_1')
        node('FH0'+label,hoop_at_z(front,.1174533215260441,side),'FrontHoop_1')
        node('FHU'+label,hoop_at_z(front,p['frontSideUpperZM'],side),'FrontHoop_1')
        node('FHB'+label,hoop_at_z(front,.640,side),'FrontHoop_1')
        x=p['cockpitStationXM'];z=np.interp(x,[nodes['MH0'+label][0],nodes['FH0'+label][0]],[nodes['MH0'+label][2],nodes['FH0'+label][2]])
        node('CS0'+label,[x,side*p['cockpitHalfWidthM'],z]);node('CSU'+label,[x,side*p['cockpitHalfWidthM'],p['sideUpperZM']])
        node('BR'+label,[p['braceRearXM'],side*p['braceRearYM'],p['braceRearZM']])
        node('BAT0'+label,[p['batteryPostXM'],side*p['braceRearYM'],.068])
        fb=src['Front Bulkhead Sketch'];node('FB0'+label,[fb[0]['start'][0],side*abs(fb[0]['start'][1]),fb[0]['start'][2]])
        node('FBU'+label,[fb[0]['start'][0],side*abs(fb[0]['start'][1]),fb[2]['start'][2]])
        points=[s['end'] for s in src['F'+label+' Offset']];fore=sorted(points,key=lambda a:a[0])[-2:];aft=sorted(points,key=lambda a:a[0])[:2]
        node('FF0'+label,min(fore,key=lambda a:a[2]));node('FFU'+label,max(fore,key=lambda a:a[2]));node('FA0'+label,min(aft,key=lambda a:a[2]));node('FAU'+label,max(aft,key=lambda a:a[2]))
        a=np.array(nodes['FF0'+label]);b=np.array(nodes['FFU'+label]);node('FFTop'+label,(a+(b-a)*(.423-a[2])/(b[2]-a[2])).tolist())
        rearpoints=[s['end'] for s in src['R'+label+' Offset']];rf=sorted(rearpoints,key=lambda a:a[0])[-2:];ra=sorted(rearpoints,key=lambda a:a[0])[:2]
        node('RF0'+label,min(rf,key=lambda a:a[2]));node('RFU'+label,max(rf,key=lambda a:a[2]));node('RA0'+label,min(ra,key=lambda a:a[2]));node('RAU'+label,max(ra,key=lambda a:a[2]))
        # F.11.1.4: the powertrain solid (Loft8) hangs below the rear bulkhead; a low node under each
        # rear lower rail (x -130 mm, z 75 mm) brings its lower-rear corner inside the envelope.
        node('PT0'+label,[-.130,nodes['RA0'+label][1],.075])
    # Source bulkhead perimeter, with real independent mitered tube bodies.
    group('FrontBulkhead','front_bulkhead','B',[('FB0R','FB0L'),('FB0L','FBUL'),('FBUL','FBUR'),('FBUR','FB0R')])
    group('FrontBulkheadDiagonal','front_bulkhead_diagonal','B',[('FB0R','FBUL')])
    # Preserve the supplied rear-bulkhead polygon and differential uprights.
    rear=src['Sketch181'];rn=[]
    for i,s in enumerate(rear[:6]):rn.append(node('RB'+str(i),s['start']))
    group('RearBulkhead','rear_bulkhead','B',[(rn[i],rn[(i+1)%6]) for i in range(6)])
    for i,s in enumerate(rear[6:]):
        node('RBD'+str(i)+'A',s['start']);node('RBD'+str(i)+'B',s['end']);group('RearUpright'+str(i),'rear_impact_upright','B',[('RBD'+str(i)+'A','RBD'+str(i)+'B')])
    for side,label in ((1,'L'),(-1,'R')):
        group('MainBrace'+label,'main_brace_and_lower_support','B',[('MH0'+label,'BR'+label),('BR'+label,'MHB'+label)])
        group('MainBraceUpperSupport'+label,'main_brace_upper_support','B',[('MHU'+label,'BR'+label)])
        group('SideLower'+label,'side_lower','B',[('MH0'+label,'CS0'+label),('CS0'+label,'FH0'+label)])
        group('SideUpper'+label,'side_upper','B',[('MHU'+label,'CSU'+label),('CSU'+label,'FHU'+label)])
        group('FrontLower'+label,'bulkhead_lower_support','C',[('FH0'+label,'FF0'+label),('FF0'+label,'FB0'+label)])
        group('FrontUpper'+label,'bulkhead_upper_support','C',[('FHU'+label,'FFTop'+label),('FFTop'+label,'FBU'+label)])
        group('FrontBrace'+label,'front_hoop_brace','B',[('FHB'+label,'FBU'+label)])
        group('SidePost'+label,'side_triangulation','B',[('CS0'+label,'CSU'+label)])
        group('SideDiagonalRear'+label,'side_diagonal','B',[('MH0'+label,'CSU'+label)])
        group('SideDiagonalFront'+label,'side_diagonal','B',[('CS0'+label,'FHU'+label)])
        group('FrontPost'+label,'bulkhead_support_triangulation','C',[('FF0'+label,'FFTop'+label)])
        group('FrontDiagonalUpper'+label,'bulkhead_support_triangulation','C',[('FAU'+label,'FFTop'+label)])
        group('FrontDiagonalMiddle'+label,'bulkhead_support_triangulation','C',[('FAU'+label,'FF0'+label)])
        group('FrontDiagonalFront'+label,'bulkhead_support_triangulation','C',[('FF0'+label,'FBU'+label)])
        group('FrontSuspensionUpper'+label,'suspension_rail','B',[('FAU'+label,'FFU'+label)])
        group('FrontSuspensionLower'+label,'suspension_rail','B',[('FA0'+label,'FF0'+label)])
        group('RearSuspensionUpper'+label,'suspension_rail',p['rearBayProfile'],[('RAU'+label,'RFU'+label)])
        group('RearSuspensionLower'+label,'suspension_rail',p['rearBayProfile'],[('RA0'+label,'RF0'+label)])
        group('RearSuspensionForwardPost'+label,'rear_triangulation','C',[('RF0'+label,'RFU'+label)])
        group('RearSuspensionDiagonal'+label,'rear_triangulation','C',[('RA0'+label,'RFU'+label)])
        group('RearUpperLink'+label,'rear_triangulation','B',[('RFU'+label,'BR'+label)])
        group('RearLowerLink'+label,'battery_protection','B',[('RF0'+label,'BAT0'+label),('BAT0'+label,'MH0'+label)])
        group('RearLowerMiterSupport'+label,'battery_protection_triangulation','B',[('BAT0'+label,'RA0'+label)])
        group('BatteryPost'+label,'battery_protection','B',[('BAT0'+label,'BR'+label)])
        group('PowertrainGuardRear'+label,'powertrain_protection','C',[('RA0'+label,'PT0'+label)])
        group('PowertrainGuardFore'+label,'powertrain_protection','C',[('PT0'+label,'RF0'+label)])
        group('RearLowerSupport'+label,'rear_triangulation','C',[('RF0'+label,'BR'+label)])
    for name,a,b,size in [('MainBase','MH0L','MH0R','B'),('CockpitFloor','CS0L','CS0R','B'),('FrontBase','FH0L','FH0R','B'),('FrontProbeFore','FF0L','FF0R','C')]:group(name,'crossbar',size,[(a,b)])
    group('PowertrainCross','powertrain_protection','C',[('PT0L','PT0R')])
    if p['floorDiagonal']:group('CockpitFloorDiagonal','floor_brace','C',[('MH0L','CS0R')])
    trims=make_trims(groups,nodes,hoop_nodes)
    extra={'parameterLimits':{k:list(v) for k,v in limits.items()}} if limits else {}
    return {'schema':1,'status':'proposed geometry; native rule/driver/trim/study gates required','parameters':p,**extra,'mainHoopPath':main_hoop,'nodes':nodes,'groups':groups,'trims':trims,'members':members,'referenceOnlyBodies':[b['name'] for b in refs],'sourceEvidenceSha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),'loadPoints':[nodes['F'+pos+level+side] for side in ('L','R') for pos in ('A','F') for level in ('0','U')],'fixturePoints':[nodes['R'+pos+level+side] for side in ('L','R') for pos in ('A','F') for level in ('0','U')],'probes':[((np.array(nodes[a])+nodes[b])/2).tolist() for a,b in [('FH0L','FH0R'),('FF0L','FF0R')]]}

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='design/frame-003.json');parser.add_argument('--parameters');parser.add_argument('--main-hoop',default='source');args=parser.parse_args();r=recipe(load(args.parameters) if args.parameters else None,main_hoop=args.main_hoop);save(ROOT/args.output,r);print(json.dumps({'path':args.output,'groups':len(r['groups']),'members':len(r['members']),'trims':len(r['trims'])}))
