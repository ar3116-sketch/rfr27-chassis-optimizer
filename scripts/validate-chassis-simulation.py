"""Validate prescribed native chassis results. No external stiffness solver is used."""
import importlib.util,math
from pathlib import Path

spec=importlib.util.spec_from_file_location('native_validation',Path(__file__).with_name('validate-native-simulation.py'))
v=importlib.util.module_from_spec(spec);spec.loader.exec_module(v)

def validate(a,recipe):
    if a['solve']!=0 or any(a[k]!=0 for k in ('rotationError','reactionError','translationError')):raise ValueError('Native result error')
    if a['softSprings'] or a['inertialRelief']:raise ValueError('Stabilization may not conceal disconnected geometry')
    if a['materialName']!='AISI 4130 Steel, normalized at 870C':raise ValueError('Prescribed material mismatch')
    expected={m['name'] for m in recipe['members']}|{'MainHoop_1','FrontHoop_1'}
    if set(a['includedBodies'])!=expected or set(a['excludedBodies'])!=set(recipe['referenceOnlyBodies']):raise ValueError('Wrong structural body set')
    for mat in a['materials']:
        if any(mat[k] for k in ('exError','gError','rhoError')) or not (190e9<=mat['youngPa']<=220e9 and 70e9<=mat['shearPa']<=90e9 and 7700<=mat['densityKgM3']<=8000):raise ValueError('Invalid native beam steel properties')
    f=next(x for x in a['export']['files'] if x['path'].lower().endswith('.dat'))
    if a['export']['code']!=0 or v.sha(f['path'])!=f['sha256']:raise ValueError('Native audit deck missing or changed')
    d=v.parse_deck(f['path']);nodes=v.rows(a['mesh']['nodes'],4);rot=v.rows(a['rotation'],4);reaction=v.rows(a['reaction'],9);trans=v.rows(a['translation'],5)
    physical={n:d['grid'][n] for n in d['physical']}
    if len(d['components'])!=1:raise ValueError(f"Disconnected native chassis mesh: {len(d['components'])} components")
    if any(v.norm(v.sub(nodes[n],p))>1e-6 for n,p in physical.items()):raise ValueError('Export differs from native solved mesh')
    # Trimmed tubes: the native joint node sits at the solver's joint centre, a few
    # mm from the drawn node. Map each requested point one-to-one to the nearest
    # constrained (fixtures) or loaded (loads) node within the joint reach.
    # Reach 10 mm (largest correct offset seen: 7 mm). It was 30 mm on the belief that the
    # front lower-aft pickup (29 mm up the front hoop leg) shares the hoop-base joint. Where a
    # tube ends at that pickup (008/009 designs) it has its own joint, and a manual pick puts
    # the load there: a 009c hand run in the SOLIDWORKS UI loaded it at 4.7 mm and gave TR
    # 3215, while the 30 mm reach had let the pipeline load the hoop base and score 3109.
    # A pick further than 10 mm is rejected so the pipeline re-picks from the side view.
    def assign(points,pool,label,reach=.01):
        out=[]
        for p in points:
            near=sorted((v.norm(v.sub(physical[n],p)),n) for n in pool)
            if not near or near[0][0]>reach:raise ValueError(f'No {label} node within {reach*1000:.0f} mm of {p}')
            out.append((near[0][1],near[0][0]))
        if len({n for n,_ in out})!=len(out):raise ValueError(f'Two {label} points share a native node')
        return out
    fixture_map=assign(recipe['fixturePoints'],[n for n in d['spc'] if n in d['physical']],'fixture')
    roots={n for n,_ in fixture_map}
    actual={n:ds for n,ds in d['spc'].items() if n in d['physical']}
    if set(actual)!=roots or any(ds!=set('123') for ds in actual.values()):raise ValueError('Fixtures do not match all eight prescribed translation-only nodes')
    # Restraints off the structure (orientation/dummy grids no CBAR or RBAR uses)
    # cannot carry load; any restraint on a structural node must be a prescribed root.
    if set(n for n in d['spc'] if n in d['physical'])!=roots:raise ValueError('Unexpected restraint on a structural node')
    # Preserve the explicitly requested reversed load case. Deck forces print to 3 decimals (1e-3 N).
    load_map=assign(recipe['loadPoints'],list(d['forces']),'load')
    expected_loads={n:[0,0,(1 if p[1]>0 else -1)*100*4.4482216152605*a['loadScale']] for (n,_),p in zip(load_map,recipe['loadPoints'])}
    if set(expected_loads)!=set(d['forces']) or any(v.norm(v.sub(d['forces'][n],q))>1e-3 for n,q in expected_loads.items()):raise ValueError('Load nodes/directions/magnitudes differ from the procedure')
    applied=[sum(q[k] for q in expected_loads.values()) for k in range(3)]
    moment=[sum(v.cross(physical[n],q)[k] for n,q in expected_loads.items()) for k in range(3)]
    rf=[sum(q[k] for q in reaction.values()) for k in range(3)]
    rm=[sum(v.cross(nodes[n],q[:3])[k]+q[4+k] for n,q in reaction.items()) for k in range(3)]
    force_residual=v.norm([x+y for x,y in zip(applied,rf)]);moment_residual=v.norm([x+y for x,y in zip(moment,rm)])
    if force_residual>1e-5*sum(v.norm(q) for q in expected_loads.values()) or moment_residual>1e-5*v.norm(moment):raise ValueError('Native chassis reaction balance failed')
    if any(v.norm(trans[n][:3])>1e-8 or v.norm(reaction[n][4:7])>1e-4 for n in roots):raise ValueError('Rear rotational freedoms or translations incorrect')
    # Require actual beam elements along every straight recipe member. Presence
    # of a body or a successful RunAnalysis call is insufficient.
    covered={}
    def joint_end(p):
        # The solver's joint node for a drawn tube end (merged joints up to 30 mm).
        dist,n=min((v.norm(v.sub(q,p)),n) for n,q in physical.items())
        return physical[n] if dist<=.03 else p
    def coverage(a_eff,b_eff):
        ab=v.sub(b_eff,a_eff);length=v.norm(ab);axis=[x/length for x in ab]
        edges=[]
        for n1,n2 in d['edges']:
            mid=[(x+y)/2 for x,y in zip(physical[n1],physical[n2])];q=v.sub(mid,a_eff);t=sum(x*y for x,y in zip(q,axis))
            distance=v.norm(v.sub(q,[t*x for x in axis]))
            edge=v.sub(physical[n2],physical[n1]);parallel=abs(sum(x*y for x,y in zip(edge,axis)))/v.norm(edge)
            # Trimmed frames: the solver moves beam ends to its joint centres (up to
            # the 29 mm merged FA0/FH0 joint), so allow 3 mm / 8 deg and demand that
            # matching beam elements span most of the tube.
            if distance<.003 and -.03<=t<=length+.03 and parallel>.99:edges.append((n1,n2,distance,v.norm(edge)*parallel))
        return edges,sum(e[3] for e in edges),length
    for m in recipe['members']:
        # Measure along the joint-adjusted line (ends moved to joint centres) and along the
        # drawn line (a trimmed end kept on its own axis and tied to an off-axis joint centre
        # by a rigid link, e.g. 009c RearSuspensionLowerR at the rear bulkhead), keep the better.
        edges,span,length=max(coverage(joint_end(m['a']),joint_end(m['b'])),coverage(m['a'],m['b']),key=lambda c:c[1]/c[2])
        if not edges or span<.7*length:raise ValueError(f"Structural tube absent from native beam mesh: {m['name']} ({span:.3f} of {length:.3f} m)")
        covered[m['name']]={'elements':len(edges),'spanFraction':span/length,'maxOffsetMm':max(e[2] for e in edges)*1000}
    probes=[]
    for p in recipe['probes']:
        near=[n for n,q in physical.items() if v.norm(v.sub(p,q))<=1e-6]
        if len(near)==1:probes.append({'pointM':p,'nodes':near,'weights':[1.0],'rxRad':rot[near[0]][0],'method':'exact native mesh node'});continue
        # A beam's rotational field is interpolated on its native mesh edge.
        matches=[]
        for n1,n2 in d['edges']:
            edge=v.sub(physical[n2],physical[n1]);w=v.sub(p,physical[n1]);den=sum(q*q for q in edge);t=sum(x*y for x,y in zip(w,edge))/den
            gap=v.norm(v.sub(w,[t*x for x in edge]))
            if gap<=1e-6 and 0<t<1:matches.append((n1,n2,t))
        if len(matches)!=1:raise ValueError('Ambiguous or absent lower-crossbar center probe')
        n1,n2,t=matches[0];probes.append({'pointM':p,'nodes':[n1,n2],'weights':[1-t,t],'rxRad':(1-t)*rot[n1][0]+t*rot[n2][0],'method':'linear interpolation on containing native beam mesh edge'})
    if any(p['rxRad']*moment[0]<=0 for p in probes):raise ValueError('Rotation signs contradict applied torque')
    return {'passed':True,'connectedComponents':1,'fixtureOffsetsMm':[o*1000 for _,o in fixture_map],'loadOffsetsMm':[o*1000 for _,o in load_map],'fixtureNodes':sorted(roots),'loadNodes':sorted(expected_loads),'memberMeshCoverage':covered,'probes':probes,
            'rxDeg':[p['rxRad']*180/math.pi for p in probes],'actualTorqueLbFt':moment[0]/1.3558179483314,'trackerTorqueLbFt':545,
            'reactionForceResidualN':force_residual,'reactionMomentResidualNm':moment_residual,'appliedMomentNm':moment,'deckSha256':f['sha256']}

def compare(default,refined,reversed_):
    mesh=max(abs(a-b)/abs(a) for a,b in zip(default['rxDeg'],refined['rxDeg']))
    reversal=max(abs(a+b)/abs(a) for a,b in zip(default['rxDeg'],reversed_['rxDeg']))
    if mesh>.05:raise ValueError(f'Mesh sensitivity exceeds 5%: {mesh}')
    if reversal>1e-5:raise ValueError(f'Load reversal mismatch: {reversal}')
    return {'meshRelativeChange':mesh,'loadReversalRelativeError':reversal,'passed':True}
