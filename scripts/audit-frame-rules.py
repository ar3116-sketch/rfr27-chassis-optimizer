"""Numerical checks for the assignment's proposed frame recipe, not vehicle approval.

Each check has its own scope. Native trims, insertion and actual driver installation
are never inferred from the presence of a centerline or from these scalar checks.
"""
import argparse,math
import numpy as np
from frame_geometry import ROOT,load,save

OD=.0254
STOCK={'A':.095*.0254,'B':.065*.0254,'C':.049*.0254}
MINIMUM={'A':(11320,173,25,2),'B':(8509,114,25,1.2),'C':(6695,91,25,1.2)}

def section(wall):
    d=OD*1000;t=wall*1000
    return {'I_mm4':math.pi/64*(d**4-(d-2*t)**4),'area_mm2':math.pi/4*(d*d-(d-2*t)**2),'od_mm':d,'wall_mm':t}

def angle(a,b):
    a=np.array(a);b=np.array(b)
    return math.degrees(math.acos(np.clip(np.dot(a,b)/np.linalg.norm(a)/np.linalg.norm(b),-1,1)))

def audit(r):
    checks=[];n={k:np.array(v) for k,v in r['nodes'].items()};m={x['name']:x for x in r['members']}
    def check(name,rule,accepted,**values):checks.append({'name':name,'rule':rule,'accepted':bool(accepted),**values})
    for size in sorted({x['size'] for x in m.values() if x['size'] in STOCK}):
        s=section(STOCK[size]);threshold=MINIMUM[size]
        check('stock_'+size,'F.3.2',all(x>=y for x,y in zip(s.values(),threshold)),measured=s,minimum=dict(zip(s,threshold)))
    # Every tube's actual profile must meet the class it is declared to provide (F.3.4.1).
    # 'NS' (Non-Structural, F.3.3) tubes count toward no requirement and are not checked.
    import beam_model
    def meets(profile,cls):
        q=beam_model.section(profile);i,a,od,w=MINIMUM[cls]
        return q['I']*1e12>=i and q['A']*1e6>=a and q['od']*1e3>=od-1e-9 and q['wall']*1e3>=w-1e-9
    weak=[x['name'] for x in m.values() if x['size'] in MINIMUM and not meets(x['profile'],x['size'])]
    check('member_profiles_meet_class','F.3.4.1',not weak,failing=weak)
    main=beam_model.main_hoop_segments(beam_model.main_hoop_path(r))
    front=load(ROOT/'design/constraint-ledger.json')['hoops'][1]['worldSegments']
    # Line-line corners on the hoop path (the source sketch has a 0.38 deg one per side). They
    # are reported, not scored: the bend-support check below covers arcs only.
    lines=[s for s in main if s['kind'] in (0,'line')];corners=[]
    for i,s in enumerate(lines):
        for t in lines[i+1:]:
            for p,q in ((s['end'],t['start']),(s['end'],t['end']),(s['start'],t['start']),(s['start'],t['end'])):
                if np.linalg.norm(np.subtract(p,q))<1e-6:
                    u=np.subtract(s['end'],s['start']);v=np.subtract(t['end'],t['start'])
                    a=angle(u,v);a=min(a,180-a)
                    if a>.01:corners.append({'pointM':list(p),'directionChangeDeg':a})
    # Native hoop audit establishes the actual surface maximum. Endpoint maxima
    # are a lower bound and cannot admit near-limit brace height tests.
    def height_interval(segments):
        low=max(s[k][2] for s in segments for k in ('start','end'))+OD/2
        high=low
        for s in segments:
            if s['kind'] in ('arc',1):
                a=np.array(s['start']);b=np.array(s['end']);c=np.array(s['center']);u=a-c;v=b-c
                ang=math.acos(np.clip(np.dot(u,v)/np.linalg.norm(u)/np.linalg.norm(v),-1,1));normal=np.cross(u,v);normal/=np.linalg.norm(normal);w=np.cross(normal,u)
                candidates=[0,ang];t=math.atan2(w[2],u[2])
                for q in (t,t+math.pi,t+2*math.pi):
                    if 0<=q<=ang:candidates.append(q)
                high=max(high,max(c[2]+u[2]*math.cos(q)+w[2]*math.sin(q)+OD/2 for q in candidates))
        return high
    main_top=height_interval(main);front_top=height_interval(front)
    # Side impact structure (F.6.4) and front bulkhead support (F.6.2.3) are found and judged from tube
    # geometry and connectivity alone (scripts/rule_structure.py), so re-routed layouts are audited too.
    import rule_structure
    structure=rule_structure.check(r)
    for side in ('L','R'):
        si=structure['sideImpact'][side];bs=structure['bulkheadSupport'][side]
        zone=si.get('zone') or si.get('bestZone') or {}
        check('side_upper_zone_'+side,'F.6.4.4',si['zoneOk'],topHeightsAboveLowerBottomM=zone.get('topsAboveLowerBottomM'),
              scope='Upper member found by geometry (rule_structure.py)')
        h=main_top-n['MHB'+side][2]
        check('main_brace_attachment_'+side,'F.5.9.4',0<=h<=.160,distanceBelowTopM=h)
        a=n['MH0'+side]-n['MHB'+side];b=n['BR'+side]-n['MHB'+side]
        included=angle(a[[0,2]],b[[0,2]])
        check('main_brace_angle_'+side,'F.5.9.5',included>=30,sideViewIncludedAngleDeg=included)
        check('main_brace_direction_'+side,'F.5.9.3',n['BR'+side][0]<n['MHB'+side][0],direction='rearward')
        check('front_brace_attachment_'+side,'F.6.3.4',0<=front_top-n['FHB'+side][2]<=.05,distanceBelowTopM=front_top-n['FHB'+side][2])
        check('front_brace_direction_'+side,'F.6.3.2',n['FBU'+side][0]>n['FHB'+side][0],direction='forward')
        detail=lambda x:{k:v for k,v in x.items() if k not in ('pass','rule')}
        check('side_impact_structure_'+side,'F.6.4.2-.6, F.1.17, F.5.2.3',si['pass'],**detail(si),
              scope='Name-free recipe geometry and connectivity; native connections verified separately')
        check('front_bulkhead_support_'+side,'F.6.2.3, F.1.17, F.5.2.3',bs['pass'],**detail(bs),
              scope='Name-free recipe geometry and connectivity; native connections verified separately')
    width=abs(n['MH0L'][1]-n['MH0R'][1])-OD
    check('main_hoop_base_width','F.5.8.4',width>=.380,insideWidthM=width)
    # Each non-collinear, two-tube regulated member needs a same-stock support
    # at its miter, within 30 degrees of the member's plane (upper SID excepted).
    for g in r['groups']:
        ms=g['members']
        if len(ms)!=2:continue
        common=set(ms[0]['nodes'])&set(ms[1]['nodes'])
        if len(common)!=1:raise ValueError('Ambiguous two-member miter')
        joint=next(iter(common));ends=[next(k for k in x['nodes'] if k!=joint) for x in ms]
        normal=np.cross(n[ends[0]]-n[joint],n[ends[1]]-n[joint]);normal/=np.linalg.norm(normal)
        supports=[]
        for x in m.values():
            if x['name'] in {z['name'] for z in ms} or joint not in x['nodes']:continue
            other=next(k for k in x['nodes'] if k!=joint);v=n[other]-n[joint]
            outplane=math.degrees(math.asin(np.clip(abs(np.dot(v,normal))/np.linalg.norm(v),0,1)))
            exempt=ms[0]['role']=='side_upper'
            if exempt or (x['profile']==ms[0]['profile'] and outplane<=30):supports.append({'member':x['name'],'angleFromPlaneDeg':outplane})
        check('miter_support_'+g['name'],'F.5.2.3',bool(supports),joint=joint,supports=supports,upperSideException=ms[0]['role']=='side_upper')
    # This family has fixed hoops. A tube footprint at an arc must be supported
    # by the side-view triangular network. No generic 25 mm weld-node exemption
    # is inferred from the rule's monocoque attachment alternative.
    for hoop,segments,braceprefix,attachprefix in [('main',main,'MHB','MHU'),('front',front,'FHB','FHU')]:
        for side in ('L','R'):
            sign=1 if side=='L' else -1
            arcs=[s for s in segments if s['kind'] in (1,'arc') and (s['start'][1]+s['end'][1])*sign>1e-8 and min(s['start'][2],s['end'][2])<n[braceprefix+side][2]]
            for i,s in enumerate(arcs):
                low=min(s['start'][2],s['end'][2]);high=max(s['start'][2],s['end'][2])
                # Footprint must reach the bend's extremum along the source path;
                # the native end-face audit verifies the physical connection.
                nearest=min([attachprefix+side,braceprefix+side],key=lambda k:max(low-n[k][2],n[k][2]-high,0))
                outside=max(low-n[nearest][2],n[nearest][2]-high,0)
                check(f'{hoop}_bend_support_{side}_{i}','F.5.6.3',outside<=OD/2,arcZIntervalM=[low,high],supportNode=nearest,distanceOutsideBendZM=float(outside),nativeConnectionRequired=True)
    for hoop,segments in [('main',main),('front',front)]:
        radii=[s.get('radiusM',s.get('radius')) for s in segments if s['kind'] in (1,'arc')]
        check(hoop+'_bend_radius','F.5.2.1',all(q>=3*OD for q in radii),radiiM=radii,minimumM=3*OD)
    main_lean=max(math.degrees(math.atan2(abs(s['end'][0]-s['start'][0]),abs(s['end'][2]-s['start'][2]))) for s in main if s['kind'] in (0,'line') and abs(s['end'][2]-s['start'][2])>1e-6)
    check('main_hoop_side_inclination','F.5.8',main_lean<10,maximumDeg=main_lean)
    # F.11.1.4 / F.1.11: the powertrain solid (Loft8, Tractive System) must lie inside the Primary
    # Structure Envelope, the hull of the outer surfaces of the rule-role members and both hoops.
    from frame_geometry import source_geometry
    from scipy.spatial import ConvexHull
    pts=[]
    def ring_pts(a,b,r):
        d=(b-a)/np.linalg.norm(b-a);u=np.cross(d,[0,0,1]) if abs(d[2])<.9 else np.cross(d,[1,0,0]);u/=np.linalg.norm(u);w=np.cross(d,u)
        return [c+r*(np.cos(t)*u+np.sin(t)*w) for t in np.linspace(0,2*np.pi,12,endpoint=False) for c in (a,b)]
    for x in m.values():
        if x['role'] in ('candidate','x_brace') or x['size']=='NS':continue
        pts+=ring_pts(np.array(x['a']),np.array(x['b']),beam_model.section(x['profile'])['od']/2)
    for poly in beam_model.hoops(beam_model.main_hoop_path(r)).values():
        poly=[np.array(p) for p in poly];pts+=[q for a,b in zip(poly,poly[1:]) if np.linalg.norm(b-a)>1e-6 for q in ring_pts(a,b,.0127)]
    eq=ConvexHull(np.array(pts)).equations;_,refs=source_geometry()
    box=np.array(next(b for b in refs if b['name']=='Loft8')['verticesM'])
    outside=float(np.max(box@eq[:,:3].T+eq[:,3]))
    check('powertrain_in_primary_envelope','F.11.1.4 (Loft8 as the motor/powertrain envelope)',outside<=0,worstOutsideMm=round(outside*1000,1))
    return {'scope':'Assignment recipe scalar and graph checks; not complete rule gate',
            'allImplementedChecksPass':all(c['accepted'] for c in checks),'checks':checks,
            'mainHoopPath':beam_model.main_hoop_path(r),'mainHoopLineCorners':corners,
            'unverified':['Native contact and circumferential fit at all trims','Continuous cockpit-template insertion','Three-circle template placement and rollover margin','Production cutting and assembly access']
                        +(['F.5.6.3: main hoop line-line corners (see mainHoopLineCorners) are not treated as bends needing a node'] if corners else []),
            'outsideScope':['Real team driver certification','Seat/pedal mechanisms','SES submission or fabrication release']}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('recipe');p.add_argument('output');a=p.parse_args();r=audit(load(a.recipe));save(a.output,r);print({'pass':r['allImplementedChecksPass'],'failed':[c['name'] for c in r['checks'] if not c['accepted']]})
