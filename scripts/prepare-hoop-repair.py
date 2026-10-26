"""Derive a tangent main hoop from the supplied sketch, preserving its plane/endpoints."""
import json,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
def save(p,v):(ROOT/p).write_text(json.dumps(v,indent=2)+'\n',encoding='utf-8')
src=json.loads((ROOT/'handoff/evidence/engineering-2026-10-11T14-18-45-377Z/stdout.json').read_text())['result']
sk=next(f['sketch'] for f in src['features'] if f['name']=='Main Roll Hoop Sketch')
def point(i,end):
 p=sk['segments'][i]['detail'][end];return np.array([-p[0],p[1]])
def intersect(a,b,c,d):
 v=b-a;w=d-c;t=np.linalg.solve(np.column_stack([v,-w]),c-a)[0];return a+t*v
base=point(12,'start')
lower=intersect(base,point(12,'end'),point(13,'start'),point(11,'end'))
upper=intersect(point(13,'start'),point(11,'end'),point(10,'end'),point(10,'start'))
top=intersect(point(10,'end'),point(10,'start'),np.array([0.,0.]),np.array([2.,0.]))
mirror=lambda p:np.array([p[0],-p[1]])
poly=[base,lower,upper,top,mirror(upper),mirror(lower),mirror(base)]
radius=3.25*.0254;fillets=[]
for i in range(1,len(poly)-1):
 v=poly[i];incoming=(v-poly[i-1]);incoming/=np.linalg.norm(incoming)
 outgoing=poly[i+1]-v;outgoing/=np.linalg.norm(outgoing)
 angle=math.acos(np.clip(incoming@outgoing,-1,1));cut=radius*math.tan(angle/2)
 a=v-incoming*cut;b=v+outgoing*cut
 cross=incoming[0]*outgoing[1]-incoming[1]*outgoing[0]
 normal=np.array([-incoming[1],incoming[0]])*np.sign(cross);center=a+normal*radius
 mid_dir=(a-center)+(b-center);mid=center+radius*mid_dir/np.linalg.norm(mid_dir)
 fillets.append({'start':a,'end':b,'center':center,'mid':mid,'angleDeg':math.degrees(angle),'radiusM':radius})
m=sk['modelToSketch'];r=np.array(m[:9]).reshape(3,3);offset=np.array(m[9:12])
def world(p):return ((np.array([-p[0],p[1],0])-offset)@np.linalg.inv(r)).tolist()
segments=[];last=poly[0];straight=[]
for f in fillets:
 length=float(np.linalg.norm(f['start']-last));straight.append(length)
 segments.append({'kind':'line','start':world(last),'end':world(f['start']),'lengthM':length})
 segments.append({'kind':'arc',**{k:world(f[k]) for k in ('start','end','center','mid')},'radiusM':radius,'angleDeg':f['angleDeg']})
 last=f['end']
length=float(np.linalg.norm(poly[-1]-last));straight.append(length)
segments.append({'kind':'line','start':world(last),'end':world(poly[-1]),'lengthM':length})
assert min(straight[1:-1])>=4*.0254 and min(straight[0],straight[-1])>=3*.0254
assert radius>=3*.0254
recipe={'schema':1,'purpose':'Replace two source centerline kinks with one tangent side per side; continuous bent main hoop',
 'sourceSketch':'Main Roll Hoop Sketch','sourcePlaneAndEndpointsPreserved':True,'sourceFilesUnchanged':True,
 'profile':'1.000 x 0.095.SLDLFP','dieRadiusIn':3.25,'bendCount':len(fillets),'straightLengthsM':straight,
 'sourceRadiiM':[.1016,.09525],'radiusChangeReason':'Use supplied VR3 3.25 inch die instead of source 4 and 3.75 inch radii',
 'designChange':'Derived working path; original sketch and candidate-001 remain available. No Boolean union used to conceal kinks.',
 'virtualCornersLocalHeightLateralM':[p.tolist() for p in poly],'segments':segments,
 'limits':['Native body continuity, actual geometry and clearance must be verified after build','Stock radius/leg checks alone do not establish complete producibility']}
save('design/main-hoop-repair-002.json',recipe)
save('scripts/jobs/repair-main-hoop.json',{'command':'repair-main-hoop',
 'source':str(ROOT/'cad/working/candidate-001/RFR27-candidate-001.SLDPRT'),
 'output':str(ROOT/'cad/working/candidate-002/RFR27-candidate-002.SLDPRT'),
 'recipe':str(ROOT/'design/main-hoop-repair-002.json')})
print(json.dumps({'bendAnglesDeg':[f['angleDeg'] for f in fillets],'straightLengthsM':straight,'endpoints':[segments[0]['start'],segments[-1]['end']]},indent=2))
