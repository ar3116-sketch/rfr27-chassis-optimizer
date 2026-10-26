"""Explicit classroom layout and conservative continuous insertion test volumes."""
import math,argparse
import numpy as np
from frame_geometry import ROOT,load,save

def layout(r):
    angle=math.radians(32);axis=np.array([-math.sin(angle),0,math.cos(angle)])
    hip=np.array([1.025,0,.235]);shoulder=hip+.490*axis;head=shoulder+.280*axis
    circles=[{'name':n,'centerM':c.tolist(),'radiusM':rad} for n,c,rad in [('RuleHip',hip,.1),('RuleShoulder',shoulder,.1),('RuleHelmet',head,.15)]]
    seat_bottom=hip[2]-.1;pedal=1.970;steering_rear=1.200;headrest_x=head[0]-.170
    # A convex hull of circumscribed circles contains both tangent connections
    # as well as the complete circles. It conservatively checks the whole 2D
    # driver template, rather than only three disconnected disks.
    points=sorted((c['centerM'][0]+c['radiusM']/math.cos(math.pi/128)*math.cos(2*math.pi*i/128),
                   c['centerM'][2]+c['radiusM']/math.cos(math.pi/128)*math.sin(2*math.pi*i/128))
                  for c in circles for i in range(128))
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    def chain(seq):
        out=[]
        for point in seq:
            while len(out)>=2 and cross(out[-2],out[-1],point)<=0:out.pop()
            out.append(point)
        return out
    hull=chain(points)[:-1]+chain(reversed(points))[:-1]
    envelopes=[{'name':'RuleDriverOutline','polygon':[[x,-.0035,z] for x,z in hull],'direction':[0,1,0],'depthM':.007,
                'test':'F.5.6.5 full template','conservativeShape':'Convex hull of circumscribed circle polygons includes the complete circles and tangent connections.'}]
    upper=min(m[k][2]+.0127 for m in r['members'] if m['role']=='side_upper' for k in ('a','b'))
    cockpit_low=min(r['nodes'][x][2] for x in ['MH0L','MH0R','CS0L','CS0R','FH0L','FH0R'])-.0127
    depth_top=min(upper-.025,cockpit_low+.320);thickness=.007;low=depth_top-thickness
    rear=.805;initial_top=1.500
    for extra,label in [(0,'RuleOpening'),(.010,'SuppliedOversizeOpening')]:
        rearhalf=(.350+extra)/2;widehalf=(.550+extra)/2;fronthalf=(.400+extra)/2
        polygon=[[rear,-rearhalf,low],[rear+.1,-widehalf,low],[rear+.6,-fronthalf,low],[rear+.6,fronthalf,low],[rear+.1,widehalf,low],[rear,rearhalf,low]]
        envelopes.append({'name':label,'polygon':polygon,'direction':[0,0,1],'depthM':initial_top-low,'test':'T.1.1','templateThicknessM':thickness,'finalTopZM':depth_top,'startTopZM':initial_top,'foreAftTranslationM':0})
    start_x=1.160;finish_x=pedal-.100;center_z=.350
    for size,label in [(.350,'RuleInternal'),(.360,'SuppliedOversizeInternal')]:
        h=size/2;c=.050
        section=[[-h,-h+c],[-h+c,-h],[h-c,-h],[h,-h+c],[h,h-c],[h-c,h],[-h+c,h],[-h,h-c]]
        polygon=[[start_x-thickness/2,y,center_z+z] for y,z in section]
        envelopes.append({'name':label,'polygon':polygon,'direction':[1,0,0],'depthM':finish_x-start_x+thickness,'test':'T.1.2','templateThicknessM':thickness,'startCenterXM':start_x,'endCenterXM':finish_x,'centerZM':center_z,
                           'conservativeShape':'Circular center notches are filled, so this solid contains the exact template.'})
        # Insertion behind the steering column before the horizontal traverse.
        # This enclosing box contains every vertical position of the template.
        envelopes.append({'name':label+'VerticalEntry','polygon':[[start_x-thickness/2,-h,center_z-h],[start_x+thickness/2,-h,center_z-h],[start_x+thickness/2,h,center_z-h],[start_x-thickness/2,h,center_z-h]],'direction':[0,0,1],'depthM':initial_top-(center_z-h),
                           'test':'T.1.2 entry','conservativeShape':'Full rectangular enclosure of the vertical insertion, behind proposed column.'})
    def top(segments):
        candidates=[]
        for s in segments:
            a=np.array(s['start']);b=np.array(s['end']);candidates.extend([a,b])
            if s['kind'] in (1,'arc'):
                c=np.array(s['center']);u=a-c;v=b-c;normal=np.cross(u,v);normal/=np.linalg.norm(normal);w=np.cross(normal,u)
                end=math.acos(np.clip(np.dot(u,v)/np.linalg.norm(u)/np.linalg.norm(v),-1,1));t=math.atan2(w[2],u[2])
                for q in (t,t+math.pi,t+2*math.pi):
                    if 0<=q<=end:candidates.append(c+u*math.cos(q)+w*math.sin(q))
        p=max(candidates,key=lambda a:a[2]).copy();p[2]+=.0127;return p
    import beam_model
    main=top(beam_model.main_hoop_segments(beam_model.main_hoop_path(r)));front=top(load(ROOT/'design/constraint-ledger.json')['hoops'][1]['worldSegments'])
    rear_brace=np.array(r['nodes']['BRL']);checks=[]
    def check(name,passed,**data):checks.append({'name':name,'accepted':bool(passed),**data})
    for name,a,b in [('main_to_front',main,front),('main_to_rear_brace',main,rear_brace)]:
        av=a[[0,2]];bv=b[[0,2]];hv=head[[0,2]];v=bv-av;t=float(np.dot(hv-av,v)/np.dot(v,v));closest=av+np.clip(t,0,1)*v
        clearance=float(np.linalg.norm(hv-closest)-.150)
        # Also require the center below the infinite rollover line (not beyond it).
        line_z=float(av[1]+(hv[0]-av[0])*v[1]/v[0])
        check('helmet_'+name,clearance>=.050 and head[2]<line_z,clearanceM=clearance,lineEndpointsM=[a.tolist(),b.tolist()],closestParameter=t)
    check('circle_dimensions',abs(np.linalg.norm(shoulder-hip)-.490)<1e-10 and abs(np.linalg.norm(head-shoulder)-.280)<1e-10,hipShoulderM=float(np.linalg.norm(shoulder-hip)),shoulderHeadM=float(np.linalg.norm(head-shoulder)))
    check('hip_to_pedal',pedal-hip[0]>=.915,distanceM=float(pedal-hip[0]))
    check('head_restraint_gap',0<=head[0]-.150-headrest_x<=.025,gapM=float(head[0]-.150-headrest_x))
    check('entry_behind_column',start_x+thickness/2<steering_rear,entryForemostXM=start_x+thickness/2,columnRearmostXM=steering_rear)
    return {'schema':1,'scope':'New-member chassis with explicit proposed layout datums; no real-team hardware certification',
            'datums':{'seatBottomZM':float(seat_bottom),'seatBackReclineDeg':32,'seatRearwardPosition':True,'pedalForemostRearFaceXM':pedal,'headRestraintFaceXM':float(headrest_x),'steeringColumnRearmostXM':steering_rear,'seatBackNormalXZ':[-math.cos(angle),-math.sin(angle)],'seatForwardLimitXM':1.125},
            'driverCircles':circles,'sweptEnvelopes':envelopes,'scalarChecks':checks,'scalarPass':all(c['accepted'] for c in checks)}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('recipe');p.add_argument('output');a=p.parse_args();save(a.output,layout(load(a.recipe)))
