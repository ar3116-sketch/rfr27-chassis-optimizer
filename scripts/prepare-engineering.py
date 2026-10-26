"""Reproducible proposed pose and native hoop jobs. No CAD/source file is overwritten."""
import json, math, hashlib, shutil
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'design';OUT.mkdir(exist_ok=True)
def write(path,value): path.write_text(json.dumps(value,indent=2)+'\n')
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def source_result(stamp): return json.loads((ROOT/'handoff/evidence'/stamp/'stdout.json').read_text())['result']

src=source_result('engineering-2026-10-11T14-18-45-377Z')
features={f['name']:f for f in src['features']}
def world(sketch,p):
    m=sketch['modelToSketch'];r=np.array(m[:9]).reshape(3,3)
    return ((np.array(p)-m[9:12])@np.linalg.inv(r)).tolist()
hoops=[]
for name,sketch,indices in [
    ('MainHoop','Main Roll Hoop Sketch',[12,4,13,11,3,10,2,9,1,8,6,0,7]),
    ('FrontHoop','Front Roll Hoop Sketch',[7,1,10,4,12,3,13,2,11,0,8])]:
    s=features[sketch]['sketch'];records=[]
    for i in indices:
        seg=s['segments'][i];det=seg['detail'];records.append({'sourceIndex':i,'kind':seg['kind'],**{k:world(s,v) if k in ('start','end','center') else v for k,v in det.items()}})
    hoops.append({'name':name,'sketch':sketch,'segments':indices,'profile':'1.000 x 0.095.SLDLFP','worldSegments':records})
candidate=ROOT/'cad/working/candidate-001/RFR27-candidate-001.SLDPRT';candidate.parent.mkdir(exist_ok=True)
starter=ROOT/'cad/working/RFR27-working-starter.SLDPRT'
if not candidate.exists():shutil.copy2(starter,candidate)
write(ROOT/'scripts/jobs/build-hoops.json',{'command':'build-hoops','path':str(candidate),'hoops':hoops})

# Joint centers measured from repeated concentric 2.58/5.95 mm-radius holes in imported solids.
joints={'hip':[-.1525,.985],'lumbar':[-.109,1.077],'abdomen':[-.070,1.314],
        'shoulder':[-.135,1.506],'neck':[-.070,1.623],'head':[-.090,1.691],
        'knee':[-.1525,.537],'ankle':[-.1525,.083],'elbow':[-.457,1.4655],'wrist':[-.738,1.4635]}
parameters={'hipX_M':1.025,'hipZ_M':.235,'backReclineDeg':32,'thighRiseDeg':12,'calfDeclineDeg':12,
            'upperArmDownDeg':52,'forearmUpDeg':14,'footForwardTiltDeg':12}
def rotation(deg):
    a=math.radians(deg);c,s=math.cos(a),math.sin(a)
    return np.array([[-c,s,0],[0,0,1],[s,c,0]]) # column-vector source-to-chassis, det +1
poses={};world_joints={'hip':np.array([parameters['hipX_M'],0,parameters['hipZ_M']])}
def part(name,anchor,angle,next_joints):
    r=rotation(angle);p=np.array([*joints[anchor],0]);t=world_joints[anchor]-r@p
    # Centre the thickness of each plate on the car Y=0 plane; joints share XZ positions.
    segment=next(s for s in percy['segments'] if name.lower() in Path(s['target']).stem.lower().split('-'))
    bb=segment['body']['boundsM'];t[1]=-(bb[2]+bb[5])/2
    mat=[*r.T.flatten(),*t,1,0,0,0]
    poses[name]={'path':segment['target'],'name':'Percy_'+name,'transform':mat,'sourceAnchor':anchor,'angleDeg':angle}
    for j in next_joints:world_joints[j]=r@np.array([*joints[j],0])+t;world_joints[j][1]=0
percy=source_result('engineering-2026-10-11T14-17-46-991Z')
back=-parameters['backReclineDeg']
part('Pelvis','hip',back,['lumbar']);part('Abdomen','lumbar',back,['abdomen'])
part('Torso','abdomen',back,['shoulder','neck']);part('Neck','neck',0,['head']);part('Helmet','head',0,[])
part('Thigh','hip',-90-parameters['thighRiseDeg'],['knee'])
part('Calf','knee',-90+parameters['calfDeclineDeg'],['ankle']);part('Foot','ankle',-parameters['footForwardTiltDeg'],[])
# Source bicep points mostly in -X; correct its small initial slope before setting target direction.
def forward_angle(start,end,target):
    d=np.array(joints[end])-joints[start];source_angle=math.degrees(math.atan2(d[1],d[0]));return 180-target-source_angle
part('Bicep','shoulder',forward_angle('shoulder','elbow',-parameters['upperArmDownDeg']),['elbow'])
part('Arm','elbow',forward_angle('elbow','wrist',parameters['forearmUpDeg']),['wrist']);part('Hand','wrist',poses['Arm']['angleDeg'],[])
identity=[1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0]
pose={'schema':1,'status':'PROPOSED DESIGN POSE — not team-supplied or clearance-approved','parameters':parameters,
      'source':'sources/supplemental/designjudges/Percy.stp','sourceSha256':sha(ROOT/'sources/supplemental/designjudges/Percy.stp'),
      'pivotBasis':'Native repeated 2.58/5.95 mm concentric holes; segment transforms are rigid, no scaling',
      'worldJointsM':{k:v.tolist() for k,v in world_joints.items()},'segments':poses,
      'limitations':['Flat 2D plate assembly, not full 3D occupancy','Seat, pedals, steering and head restraint are proposed references, not supplied mechanisms','Must check independent F.5.6 circle template and insertion trajectory']}
write(OUT/'percy-pose-001.json',pose)
write(ROOT/'scripts/jobs/pose-percy.json',{'command':'pose-assembly','output':str(ROOT/'cad/working/percy/RFR27-Percy-seated-001.SLDASM'),
      'poseDefinition':str(OUT/'percy-pose-001.json'),'components':[{k:v for k,v in p.items() if k in ('path','name','transform')} for p in poses.values()]})
write(ROOT/'scripts/jobs/integrate-percy.json',{'command':'pose-assembly','output':str(ROOT/'cad/working/candidate-001/RFR27-driver-layout-001.SLDASM'),
      'poseDefinition':str(OUT/'percy-pose-001.json'),'components':[
          {'path':str(candidate),'name':'Chassis_candidate_001','transform':identity,'configuration':'Default<As Machined>'},
          {'path':str(ROOT/'cad/working/percy/RFR27-Percy-seated-001.SLDASM'),'name':'Percy_proposed_pose','transform':identity}]})
ledger={'schema':1,'units':'m, N, kg, radians unless explicit','coordinates':{'X':'forward','Y':'left','Z':'up'},
        'starterSha256':sha(starter),'protectedSketches':['FL Offset','FR Offset','RL Offset','RR Offset'],
        'protectedPlanes':['FUL','FUR','FLL','FLR','RUL','RUR','RLL','RLR'],'hoops':hoops,
        'referenceOnlyBodies':['Boss-Extrude1','Boss-Extrude2','Boss-Extrude3','Loft7','Loft8'],
        'loadN':444.82216152605,'trackerTorqueLbFt':545,'rearTranslationConstraint':['X','Y','Z'],'rearRotationConstraint':[],
        'physicalGapMaxM':.00127,'preferredGapBelowM':.000254,'jointToleranceStrictlyBelowM':.00127,
        'authoritativeFinalEvaluator':'native trimmed SolidWorks + prescribed Simulation; screening cannot admit a winner',
        'pendingHardGates':['rule coverage','continuous hoops','native joint trims','cockpit insertion sweep','Percy and full 3D driver','keepout intersections','manufacturing access','native study and material','structural-only mass','reactions and RX probe mapping']}
write(OUT/'constraint-ledger.json',ledger)
print(json.dumps({'candidate':str(candidate),'pose':str(OUT/'percy-pose-001.json'),'worldJointsM':pose['worldJointsM']},indent=2))
