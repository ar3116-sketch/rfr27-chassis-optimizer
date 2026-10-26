"""Name-free F.6.4 / F.6.2.3 predicates (rule_structure.py) and the free-skeleton search mode."""
import copy, random, sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import frame_geometry as fg
import rule_structure as rs

B, C = '1.000 x 0.065.sldlfp', '1.000 x 0.049.sldlfp'
BASE = fg.recipe()
N = BASE['nodes']


def variant(drop=(), add=(), change=None, recipe=BASE):
    """Copy of the recipe: members whose names start with any `drop` prefix removed, (name, a, b,
    profile) members added between recipe nodes on both sides (an L/R pattern in `add`)."""
    r = copy.deepcopy(recipe)
    r['members'] = [m for m in r['members'] if not any(m['name'].startswith(d) for d in drop)]
    for name, a, b, prof in add:
        for s in 'LR':
            r['members'].append({'name': name + s, 'a': N[a + s], 'b': N[b + s], 'profile': prof, 'nodes': [a + s, b + s],
                                 'role': 'test', 'size': rs.level(prof) or 'NS'})
    for m in r['members']:
        if change and m['name'] in change: m['profile'] = change[m['name']]
    return r


def verdict(r):
    c = rs.check(r)
    return {k: {s: c[k][s]['pass'] for s in 'LR'} for k in c}, c


ALL = {'sideImpact': {'L': True, 'R': True}, 'bulkheadSupport': {'L': True, 'R': True}}


class PredicateTests(unittest.TestCase):
    def test_codex_frame_passes_and_members_are_found_by_geometry(self):
        v, c = verdict(BASE)
        self.assertEqual(v, ALL)
        si = c['sideImpact']['L']
        self.assertEqual(sorted(si['upper']), ['SideUpperL_1', 'SideUpperL_2'])
        self.assertEqual(sorted(si['lower']), ['SideLowerL_1', 'SideLowerL_2'])
        self.assertEqual(si['bays'], 4)
        self.assertEqual(c['bulkheadSupport']['R']['bays'], 5)

    def test_names_and_roles_do_not_matter(self):
        r = copy.deepcopy(BASE); rng = random.Random(3)
        for m in r['members']:
            m['name'] = 'T%06d' % rng.randrange(10 ** 6); m['role'] = 'x'; m['size'] = 'NS'
        r['nodes'] = {}
        self.assertEqual(verdict(r)[0], ALL)

    def test_missing_diagonal_leaves_a_bay_untriangulated(self):
        v, c = verdict(variant(drop=('SideDiagonalRear',)))
        self.assertFalse(v['sideImpact']['L'])
        self.assertIn('triangulated', c['sideImpact']['L']['reason'])
        self.assertTrue(v['bulkheadSupport']['L'])

    def test_warren_side_panel_without_post_passes(self):
        r = variant(drop=('SideLower', 'SidePost', 'SideDiagonalFront', 'CockpitFloor'),
                    add=[('LowerStraight', 'MH0', 'FH0', B), ('DiagonalFront', 'CSU', 'FH0', B)])
        self.assertEqual(verdict(r)[0], ALL)

    def test_size_c_diagonal_does_not_count_for_side_impact(self):
        v, _ = verdict(variant(change={'SideDiagonalRearL': C, 'SideDiagonalRearR': C}))
        self.assertEqual(v['sideImpact'], {'L': False, 'R': False})

    def test_kinked_lower_member_needs_same_size_support(self):
        # F.5.2.3: SideDiagonalFront (28.8 deg from the lower member's plane) and the cockpit floor
        # crossbar support the CS0 kink only while they have the member's OD and wall.
        r = variant(drop=('CockpitFloor',), change={'SideDiagonalFrontL': '1.000 x 0.083.sldlfp', 'SideDiagonalFrontR': '1.000 x 0.083.sldlfp'})
        v, c = verdict(r)
        self.assertFalse(v['sideImpact']['L'])
        self.assertIn('F.5.2.3', c['sideImpact']['L']['reason'])
        self.assertEqual(verdict(variant(drop=('CockpitFloor',)))[0], ALL)

    def test_upper_member_outside_the_zone_fails(self):
        r = copy.deepcopy(BASE)
        for m in r['members']:
            for k in ('a', 'b'):
                if m[k] in (N['CSUL'], N['CSUR']): m[k] = [m[k][0], m[k][1], m[k][2] + 0.06]
        v, c = verdict(r)
        self.assertFalse(v['sideImpact']['L'])
        self.assertFalse(c['sideImpact']['L']['zoneOk'])

    def test_bulkhead_support_alternatives(self):
        self.assertFalse(verdict(variant(drop=('FrontDiagonalFront',)))[0]['bulkheadSupport']['L'])
        self.assertEqual(verdict(variant(drop=('FrontDiagonalFront',), add=[('BulkheadDiagonal', 'FFTop', 'FB0', C)]))[0], ALL)
        low = variant(drop=('FrontUpper', 'FrontDiagonalFront'),
                      add=[('UpperA', 'FHU', 'FFTop', C), ('UpperB', 'FFTop', 'FB0', C)])   # meets the bulkhead at its base
        v, c = verdict(low)
        self.assertFalse(v['bulkheadSupport']['L'])
        self.assertIn('F.6.2.3.a', c['bulkheadSupport']['L']['reason'])


class FreeSkeletonTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import ground_structure as gs
        cls.gs = gs
        cls.pb = gs.Problem(BASE, gs.CATALOG['vr3'], skeleton='free')
        cls.st = cls.pb.base_state()
        cls.idx = {g['name']: i for i, g in enumerate(cls.pb.groups)}

    def pair(self, name):
        g = self.idx[name]; return g, self.pb.mirror[g]

    def test_codex_skeleton_is_released_but_legal(self):
        pb = self.pb
        self.assertFalse(pb.groups[self.idx['Weld_SidePostL']]['required'])
        self.assertTrue(pb.groups[self.idx['Weld_FrontBrace' + 'L']]['required'])
        self.assertTrue(pb.panels_ok(self.st))

    def test_predicates_gate_moves(self):
        pb, st = self.pb, self.st
        g, m = self.pair('Weld_SideDiagonalRearL')
        trial = list(st); trial[g] = trial[m] = None
        self.assertFalse(pb.valid_change(st, trial, {g, m}))
        g, m = self.pair('Weld_Cand_FB0L_FFTopL')
        ch = pb.close(st, {g: C, m: C})
        self.assertIsNone(ch.get(self.idx['Weld_FrontDiagonalFrontL'], 'kept'))     # displaced
        trial = list(st)
        for k, p in ch.items(): trial[k] = p
        self.assertTrue(pb.valid_change(st, trial, set(ch)))

    def test_fixed_mode_unchanged(self):
        pb = self.gs.Problem(BASE, self.gs.CATALOG['vr3'])
        self.assertFalse(pb.free)
        self.assertTrue(pb.groups[[g['name'] for g in pb.groups].index('Weld_SidePostL')]['required'])


if __name__ == '__main__':
    unittest.main()
