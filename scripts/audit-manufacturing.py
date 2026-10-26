"""New-member geometry/manufacturing checks; not a vendor fabrication release."""
import math
from frame_geometry import ROOT,load

def audit(recipe,native,fit):
    checks=[]
    def check(name,passed,**data):checks.append({'name':name,'accepted':bool(passed),**data})
    members={m['name']:m for m in recipe['members']};order={m['name']:i for i,m in enumerate(recipe['members'])}
    check('one_valid_solid_per_tube',native['bodyValidity'] and len(native['bodies'])==len(members)+2 and {b['name'] for b in native['bodies']}==set(members)|{'MainHoop_1','FrontHoop_1'})
    check('no_tube_intersections',native['tubeIntersectionsFree'])
    check('all_end_boundaries_fit',fit['allStraightMemberEndsFit'],maximumGapM=fit['maximumAllowedGapM'])
    check('layered_trim_order',all(order.get(t['carrier'],-1)<order[t['target']] for t in recipe['trims']))
    check('nominal_gap',all(0<=t['gapM']<.000254 for t in recipe['trims']),maximumM=max(t['gapM'] for t in recipe['trims']))
    main=load(ROOT/'design/main-hoop-repair-002.json')
    check('main_bend_stock_and_die',main['profile']=='1.000 x 0.095.SLDLFP' and main['dieRadiusIn'] in (3.25,5.88))
    lengths=main['straightLengthsM']
    check('main_bend_grips',min(lengths[0],lengths[-1])>=.0762 and min(lengths[1:-1])>=.1016,straightLengthsM=lengths,endMinimumM=.0762,betweenBendMinimumM=.1016)
    front=load(ROOT/'design/constraint-ledger.json')['hoops'][1]
    lines=[math.dist(s['start'],s['end']) for s in front['worldSegments'] if s['kind']==0]
    radii=[s['radius'] for s in front['worldSegments'] if s['kind']==1]
    check('front_bend_stock_die_and_grips',front['profile']=='1.000 x 0.095.SLDLFP' and all(min(abs(r-.08255),abs(r-.149352))<1e-7 for r in radii) and min(lines)>=.1016,straightLengthsM=lines,radiiM=radii)
    joints={}
    for m in members.values():
        for n in m['nodes']:joints.setdefault(n,[]).append(m['name'])
    return {'scope':'Assignment solid geometry, stock, bend tooling/grips, complete end fit and trim precedence',
            'geometryAccepted':all(c['accepted'] for c in checks),'checks':checks,
            'metrics':{'straightTubeCount':len(members),'bentTubeCount':2,'copeOperationCount':len(recipe['trims']),
                       'stockSections':sorted({m['profile'] for m in members.values()}|{main['profile']}),'crowdedNodes':{n:ms for n,ms in joints.items() if len(ms)>=5}},
            'reviewItems':['Weld reach and jig/subassembly sequence at the listed crowded nodes','Vendor-specific toolpath preparation'],
            'outsideScope':'Fabrication release, weld-process qualification and actual vehicle certification'}
