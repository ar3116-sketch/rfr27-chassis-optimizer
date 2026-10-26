"""Audit native acceptance results and their exported deck; never compute a chassis score."""
from pathlib import Path
import argparse, hashlib, json, math, re

def load_result(path):
    doc=json.loads(Path(path).read_text(encoding='utf-8-sig'))
    if not doc.get('ok'):raise ValueError('Native operation failed')
    a=doc['result']
    if a['solve']!=0 or any(a[k]!=0 for k in ('rotationError','reactionError')):raise ValueError('Native solve/results error')
    if a.get('cadSha256') and sha(a['path'])!=a['cadSha256']:raise ValueError('CAD hash changed since solve')
    return a

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def norm(v):return math.sqrt(sum(x*x for x in v))
def sub(a,b):return [x-y for x,y in zip(a,b)]
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def rows(values,stride):
    if len(values)%stride:raise ValueError('Unexpected native array stride')
    data={}
    for i in range(0,len(values),stride):
        row=values[i:i+stride];key=int(row[0])
        if key!=row[0] or key in data or not all(math.isfinite(v) for v in row):raise ValueError('Invalid native node result')
        data[key]=row[1:]
    return data
def near_node(nodes,p,tol=1e-6):
    found=[n for n,q in nodes.items() if norm(sub(q,p))<=tol]
    if len(found)!=1:raise ValueError(f'Ambiguous or absent mesh node at {p}: {found}')
    return found[0]

def beam_test(a):
    nodes=rows(a['mesh']['nodes'],4);r=rows(a['rotation'],4);f=rows(a['reaction'],9)
    root=near_node(nodes,[0,0,0]);tip=near_node(nodes,[1,0,0])
    od=.0254;inside=od-2*.065*.0254;j=math.pi/32*(od**4-inside**4)
    expected=10/(a['material']['shear']*j);actual=r[tip][0];relative=abs(actual-expected)/expected
    if a['material']['materialError'] or relative>1e-5 or norm(r[root])>1e-9:raise ValueError('Beam torsion analytical acceptance failed')
    reaction=[sum(v[k] for v in f.values()) for k in (0,1,2,4,5,6)]
    if norm(sub(reaction,[0,0,0,-10,0,0]))>1e-5:raise ValueError('Beam reaction balance failed')
    return {'passed':True,'cad':a['path'],'cadSha256':sha(a['path']),'rootNode':root,'tipNode':tip,'appliedTorqueNm':10,
            'polarMomentM4':j,'expectedTipRxRad':expected,'nativeTipRxRad':actual,'relativeError':relative,'reactionForceAndMoment':reaction}

def parse_deck(path):
    """Strict subset emitted by the accepted native free-field beam export."""
    grid={};edges=[];orientation=set();spc={};forces={};other=[];rigid=[]
    def number(v):return float(re.sub(r'(?<=[0-9.])([+-]\d+)$',r'e\1',v) if not re.search('[eEdD]',v) else v.replace('D','E').replace('d','e'))
    for raw in Path(path).read_text(errors='strict').splitlines():
        fields=[s.strip() for s in raw.split(',')];kind=fields[0]
        if kind=='GRID':
            if fields[2] not in ('','0'):raise ValueError('Non-global GRID coordinates need explicit transformation support')
            grid[int(fields[1])]=[number(x) for x in fields[3:6]]
        elif kind=='CBAR':
            edges.append((int(fields[3]),int(fields[4])))
            # Orientation is either a G0 node id or an explicit x1,x2,x3 vector.
            if re.fullmatch(r'\d+',fields[5]):orientation.add(int(fields[5]))
        elif kind=='RBAR':
            # Rigid joint offsets the native mesher inserts between beam ends.
            edges.append((int(fields[2]),int(fields[3])));rigid.append((int(fields[2]),int(fields[3])))
        elif kind=='SPC':
            node=int(fields[2]);dofs=set(fields[3])
            if number(fields[4])!=0:raise ValueError('Nonzero prescribed displacement unsupported')
            spc.setdefault(node,set()).update(dofs)
        elif kind=='FORCE':
            if fields[3]!='0':raise ValueError('Non-global force coordinates need transformation support')
            node=int(fields[2]);v=[number(fields[4])*number(x) for x in fields[5:8]]
            forces[node]=[x+y for x,y in zip(forces.get(node,[0,0,0]),v)]
        elif kind in ('MOMENT','CBEAM','RBE2','RBE3','SPC1','MPC','GRAV','PLOAD','PLOAD1'):other.append(kind)
    if other:raise ValueError('Unsupported load/constraint/element cards: '+str(other))
    if not grid or not edges:raise ValueError('Empty native beam deck')
    physical=set(n for edge in edges for n in edge);adj={n:set() for n in physical}
    for a,b in edges:adj[a].add(b);adj[b].add(a)
    unseen=set(physical);components=[]
    while unseen:
        seed=next(iter(unseen));todo=[seed];connected=set()
        while todo:
            n=todo.pop()
            if n in connected:continue
            connected.add(n);todo.extend(adj[n]-connected)
        components.append(connected);unseen-=connected
    return {'grid':grid,'physical':physical,'orientation':orientation,'spc':spc,'forces':forces,'components':components,'edges':[e for e in edges if e not in set(rigid)],'rigid':rigid}

def frame_test(a):
    deckfile=next(f for f in a['export']['files'] if f['path'].lower().endswith('.dat'))
    if a['export']['code']!=0 or sha(deckfile['path'])!=deckfile['sha256']:raise ValueError('Native audit export failed or changed')
    d=parse_deck(deckfile['path'])
    if len(d['components'])!=1:raise ValueError(f"Disconnected native beam mesh: {len(d['components'])} components")
    nodes=rows(a['mesh']['nodes'],4);rotation=rows(a['rotation'],4);reaction=rows(a['reaction'],9);translation=rows(a['translation'],5)
    physical={n:d['grid'][n] for n in d['physical']}
    if any(norm(sub(nodes[n],p))>1e-6 for n,p in physical.items()):raise ValueError('Exported deck and solved mesh differ')
    roots={near_node(physical,p) for p in a['rearFixturePointsM']}
    actual_spc={n:v for n,v in d['spc'].items() if n in d['physical']}
    if set(actual_spc)!=roots or any(v!=set('123') for v in actual_spc.values()):raise ValueError('Rear fixture is not exactly translation-only at the prescribed nodes')
    extra=set(d['spc'])-d['physical']
    if not extra.issubset(d['orientation']):raise ValueError('Unexpected restrained nonstructural node')
    expected_loads={near_node(physical,l['pointM']):l['forceN'] for l in a['loads']}
    if set(expected_loads)!=set(d['forces']) or any(norm(sub(d['forces'][n],v))>1e-6 for n,v in expected_loads.items()):raise ValueError('Wrong loaded nodes, directions or magnitudes')
    applied=[0.,0.,0.];torque=[0.,0.,0.];rf=[0.,0.,0.];rm=[0.,0.,0.]
    for n,v in expected_loads.items():
        applied=[x+y for x,y in zip(applied,v)];torque=[x+y for x,y in zip(torque,cross(physical[n],v))]
    for n,v in reaction.items():
        rf=[x+y for x,y in zip(rf,v[:3])];m=[x+y for x,y in zip(cross(nodes[n],v[:3]),v[4:7])];rm=[x+y for x,y in zip(rm,m)]
    if norm([x+y for x,y in zip(applied,rf)])>1e-4 or norm([x+y for x,y in zip(torque,rm)])>1e-4:raise ValueError('Native reaction balance failed')
    if any(norm(translation[n][:3])>1e-8 or norm(reaction[n][4:7])>1e-6 for n in roots):raise ValueError('Incorrect rear translation/rotation restraints')
    rx=[rotation[n][0] for n in expected_loads]
    if not all(x*torque[0]>0 for x in rx) or max(rx)-min(rx)>1e-6:raise ValueError('Inconsistent front rotation signs or symmetry')
    return {'passed':True,'cad':a['path'],'cadSha256':sha(a['path']),'deck':deckfile['path'],'deckSha256':deckfile['sha256'],
            'connectedComponents':1,'physicalNodeCount':len(physical),'beamElementCount':len(d['edges']),
            'fixtureNodes':sorted(roots),'fixtureDofs':'123 only; rotations free','loadNodes':sorted(expected_loads),
            'appliedForceN':applied,'appliedMomentNm':torque,'reactionForceN':rf,'reactionMomentNm':rm,
            'frontRxRad':rx,'meanFrontRxRad':sum(rx)/len(rx),'loadScale':a['loadScale'],'nativeNodeMappingVerified':True}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--beam',required=True);ap.add_argument('--frame',required=True);ap.add_argument('--reverse',required=True);ap.add_argument('--output',required=True);args=ap.parse_args()
    beam=beam_test(load_result(args.beam));positive=frame_test(load_result(args.frame));negative=frame_test(load_result(args.reverse))
    sign_error=abs(positive['meanFrontRxRad']+negative['meanFrontRxRad'])/abs(positive['meanFrontRxRad'])
    if sign_error>1e-5:raise ValueError('Load-reversal rotation check failed')
    result={'passed':True,'scope':'Native solver and API acceptance fixtures only; not full chassis validation or score',
            'beam':beam,'frame':positive,'reversedFrame':negative,'loadReversalRelativeError':sign_error,
            'evidenceInputs':[{ 'path':str(Path(p).resolve()),'sha256':sha(p)} for p in (args.beam,args.frame,args.reverse)]}
    Path(args.output).write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
