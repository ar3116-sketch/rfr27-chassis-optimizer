"""Checks the fast beam model against closed-form beam theory and itself."""
import json, math, sys, unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import beam_model as bm


def single(a, b, profile, fixed_node, load_node, F):
    f = bm.Frame(); i = f.node(a); j = f.node(b)
    s = bm.section(profile); f.elems.append((i, j, s, 'tube')); f.xyz = np.array(f.xyz)
    n = 2; K = np.zeros((12, 12))
    T = bm._rotation(f.xyz[i], f.xyz[j]); K += T.T @ bm._k_local(np.linalg.norm(f.xyz[j] - f.xyz[i]), s) @ T
    free = np.setdiff1d(np.arange(12), np.arange(6 * fixed_node, 6 * fixed_node + 6))
    Fv = np.zeros(12); Fv[6 * load_node:6 * load_node + 6] = F
    u = np.zeros(12); u[free] = np.linalg.solve(K[np.ix_(free, free)], Fv[free])
    return u, s


class BeamModelTests(unittest.TestCase):
    def test_section_matches_rule_table(self):
        s = bm.section('1.000 x 0.095.SLDLFP')
        self.assertAlmostEqual(s['A'] * 1e6, 174.3, delta=0.2)
        self.assertAlmostEqual(s['I'] * 1e12, 11637, delta=5)

    def test_cantilever_bending_any_orientation(self):
        L, P = 1.0, 100.0
        for b in ([1, 0, 0], [0, 1, 0], [0, 0, 1], [0.6, 0.8, 0]):
            b = np.array(b, float) * L
            perp = np.cross(b, [0, 0, 1]) if abs(b[2]) < .9 else np.array([1.0, 0, 0])
            perp /= np.linalg.norm(perp)
            u, s = single([0, 0, 0], b, '1.000 x 0.065', 0, 1, list(P * perp) + [0, 0, 0])
            expected = P * L**3 / (3 * bm.E * s['I']) + P * L / (bm.SHEAR_K * bm.G * s['A'])  # Timoshenko
            self.assertAlmostEqual(u[6:9] @ perp / expected, 1, places=9)

    def test_torsion(self):
        L, T = 1.0, 10.0
        u, s = single([0, 0, 0], [L, 0, 0], '1.000 x 0.065', 0, 1, [0, 0, 0, T, 0, 0])
        self.assertAlmostEqual(u[9] / (T * L / (bm.G * s['J'])), 1, places=9)

    def test_frame_005_connected_and_reversible(self):
        r = json.loads((bm.ROOT / 'design/frame-005.json').read_text(encoding='utf-8-sig'))
        f = bm.build(r)
        a, b = bm.solve(f, r), bm.solve(f, r, load_scale=-1)
        self.assertIsNotNone(a)
        for x, y in zip(a['rxDeg'], b['rxDeg']): self.assertAlmostEqual(x, -y, places=9)
        self.assertAlmostEqual(a['actualTorqueLbFt'], 545, delta=5)  # tracker convention vs real geometry

    def test_layout_engine_matches_recipe_model(self):
        # The layout engine models every candidate at once. Inactive X-brace candidates must not
        # capture an active X-brace's crossing node (before 20 Oct the engine scored 011b's tubes 4.7 % low).
        import frame_geometry as fg, ground_structure as gs, layout_engine as le
        r = json.loads((bm.ROOT / 'design/frame-011b.json').read_text(encoding='utf-8-sig'))
        pb = gs.Problem(fg.recipe(r['parameters']), gs.CATALOG['vr3'])
        group = {name: gi for gi, g in enumerate(pb.groups) for name, *_ in g['members']}
        cat = {c.lower(): c for c in pb.catalog}
        st = [None] * len(pb.groups)
        for m in r['members']: st[group[m.get('optimizerName', m['name'])]] = cat[m['profile'].lower()]
        eng = le.Engine(pb); eng.factor(st)
        self.assertAlmostEqual(eng.score()['tr'] / bm.evaluate(r)['trLbFtPerDeg'], 1, delta=0.005)


if __name__ == '__main__':
    unittest.main()
