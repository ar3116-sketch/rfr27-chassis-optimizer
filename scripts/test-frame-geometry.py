import copy,importlib.util,math,unittest
from frame_geometry import ROOT,recipe,source_geometry,make_trims,cope_gap,COPE_GAP_M,SMALL_TUBE_COPE_GAP_M

spec=importlib.util.spec_from_file_location('frame_rules',ROOT/'scripts/audit-frame-rules.py')
rules=importlib.util.module_from_spec(spec);spec.loader.exec_module(rules)

class FrameGeometryTests(unittest.TestCase):
    def test_source_attachment_points_are_not_moved(self):
        r=recipe();src,_=source_geometry()
        expected=[s['end'] for side in ('L','R') for s in src['F'+side+' Offset']]
        self.assertEqual(sorted(r['loadPoints']),sorted(expected))
        expected=[s['end'] for side in ('L','R') for s in src['R'+side+' Offset']]
        self.assertEqual(sorted(r['fixturePoints']),sorted(expected))

    def test_parameter_rejections(self):
        for p in ({'braceRearXM':math.nan},{'sideUpperZM':.9},{'cockpitHalfWidthM':True},
                  {'floorDiagonal':'false'},{'rearBayProfile':'A'},{'unknown':1}):
            with self.subTest(p=p),self.assertRaises(ValueError):recipe(p)

    def test_topology_choice_changes_geometry(self):
        r=recipe({'floorDiagonal':False})
        self.assertNotIn('CockpitFloorDiagonal',{m['name'] for m in r['members']})
        self.assertNotEqual(recipe()['nodes']['CS0L'],recipe({'cockpitHalfWidthM':.32})['nodes']['CS0L'])

    def test_recipe_identity_and_trim_order(self):
        r=recipe();members={m['name']:i for i,m in enumerate(r['members'])}
        self.assertEqual(len(members),len(r['members']))
        for t in r['trims']:
            self.assertIn(t['target'],members)
            self.assertLess(members.get(t['carrier'],-1),members[t['target']])

    def test_scalar_rule_violation_rejected(self):
        r=recipe();self.assertTrue(rules.audit(r)['allImplementedChecksPass'])
        for m in r['members']:
            if m['role']=='side_upper':m['a']=list(m['a']);m['a'][2]=.5
        a=rules.audit(r)
        self.assertFalse(a['allImplementedChecksPass'])
        self.assertTrue(any(c['name'].startswith('side_upper_zone') and not c['accepted'] for c in a['checks']))

    def test_small_tube_copes_touch_for_simulation_joints(self):
        # Copes involving a 0.750 in tube must stay under SOLIDWORKS's touching tolerance
        # (~0.095 mm); every gap stays inside the procedure's preferred 0.01 in.
        r=recipe();small={'name':'X','profile':'0.750 x 0.035.sldlfp','nodes':['P','Q'],'a':[0,0,0],'b':[.3,0,0]}
        self.assertEqual(cope_gap(small,{'name':'FrontHoop_1','hoop':True}),SMALL_TUBE_COPE_GAP_M)
        self.assertEqual(cope_gap(r['members'][0],small),SMALL_TUBE_COPE_GAP_M)
        self.assertEqual(cope_gap(r['members'][0],r['members'][1]),COPE_GAP_M)
        self.assertLess(SMALL_TUBE_COPE_GAP_M,.00008)
        self.assertTrue(all(0<t['gapM']<.000254 for t in r['trims']))

    def test_brace_coped_first_onto_the_kink_member_it_sits_on(self):
        # Rail kinked 3.7 deg at N (two mitered members of one group); a 0.750 in brace meets
        # N at 33.7 deg to the forward member, so its footprint lies wholly past the miter.
        k=math.radians(3.7);t=math.radians(33.7)
        nodes={'A':[-.3,0,0],'N':[0,0,0],'B':[.3*math.cos(k),-.3*math.sin(k),0],'X':[.3*math.cos(t+k),-.3*math.sin(t+k),0]}
        rail={'name':'Weld_Rail','profile':'1.000 x 0.049.sldlfp','members':[
            {'name':'Rail_1','a':nodes['A'],'b':nodes['N'],'nodes':['A','N'],'profile':'1.000 x 0.049.sldlfp'},
            {'name':'Rail_2','a':nodes['N'],'b':nodes['B'],'nodes':['N','B'],'profile':'1.000 x 0.049.sldlfp'}]}
        brace={'name':'Weld_Brace','profile':'0.750 x 0.035.sldlfp','members':[
            {'name':'Brace','a':nodes['X'],'b':nodes['N'],'nodes':['X','N'],'profile':'0.750 x 0.035.sldlfp'}]}
        trims=[x for x in make_trims([rail,brace],nodes,{}) if x['target']=='Brace']
        self.assertEqual([x['carrier'] for x in trims],['Rail_2','Rail_1'])
        self.assertTrue(all(x['gapM']==SMALL_TUBE_COPE_GAP_M for x in trims))
        # A perpendicular post straddles the miter: original weld order is kept.
        nodes['X']=[0,0,.3];brace['members'][0]['a']=nodes['X']
        self.assertEqual([x['carrier'] for x in make_trims([rail,brace],nodes,{}) if x['target']=='Brace'],['Rail_1','Rail_2'])

    def test_driver_template_and_native_checks_not_inferred(self):
        a=rules.audit(recipe())
        self.assertIn('Continuous cockpit-template insertion',a['unverified'])
        self.assertNotIn('rules',a)

if __name__=='__main__':unittest.main()
