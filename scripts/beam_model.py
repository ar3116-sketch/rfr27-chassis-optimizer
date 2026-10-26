"""Fast 3D frame (beam) model of a chassis recipe for screening layouts.

Reproduces the team TR procedure on centerlines: 4130 steel beams, the eight rear
suspension nodes fixed in translation only, +/-100 lbf on the four front suspension
nodes per side (left up), RX probed at the centres of the front lower crossbars.
Joints are rigid where tube surfaces touch (centreline gap <= r1 + r2 + 0.05 in),
like SolidWorks "treat as joint for clearance < 0.05 in".

This is a screening model. SolidWorks results on the trimmed CAD stay the score
of record; scripts/test-beam-model.py compares the two.
"""
from pathlib import Path
import json, math, re
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
E, G, RHO = 205e9, 80e9, 7850.0          # AISI 4130 normalized (SolidWorks library)
LBF, LBFT, IN = 4.4482216152605, 1.3558179483314, 0.0254
TRACKER_TORQUE_LBFT = 545.0               # 2027 TR tracker convention
JOINT_CLEARANCE = 0.05 * IN
# Points closer than this are one joint (SolidWorks merges beam joints within its
# clearance). It also prevents micro-elements, which ruin matrix conditioning.
NODE_MERGE = 0.010


def section(profile):
    """Area, second moment, torsion constant and outer size from a VR3 profile name."""
    m = re.match(r'(\d+\.\d+)\s*(SQ)?\s*x\s*(\d*\.\d+)', Path(profile).name, re.I)
    if not m: raise ValueError('Unrecognised profile ' + profile)
    D, t, square = float(m[1]) * IN, float(m[3]) * IN, bool(m[2])
    d = D - 2 * t
    if square:
        A = D * D - d * d; I = (D**4 - d**4) / 12
        J = 2 * t * t * (D - t)**4 / (2 * (D - t) * t)  # thin-walled closed square: 4A_m^2 t / perimeter
    else:
        A = math.pi / 4 * (D * D - d * d); I = math.pi / 64 * (D**4 - d**4); J = 2 * I
    return {'profile': Path(profile).name, 'od': D, 'wall': t, 'square': square, 'A': A, 'I': I, 'J': J,
            'kgPerM': A * RHO}


# ---------------------------------------------------------------- hoop geometry
def _arc_points(s, n=8):
    c = np.array(s['center']); a = np.array(s['start']) - c; b = np.array(s['end']) - c
    r = np.linalg.norm(a); ang = math.acos(np.clip(a @ b / (r * np.linalg.norm(b)), -1, 1))
    k = np.cross(a, b); k = k / np.linalg.norm(k)
    pts = []
    for i in range(n + 1):
        th = ang * i / n
        v = a * math.cos(th) + np.cross(k, a) * math.sin(th) + k * (k @ a) * (1 - math.cos(th))
        pts.append(c + v)
    return pts


def hoop_polyline(segments):
    """Chain unordered line/arc segments into one ordered polyline (metres)."""
    pieces = []
    for s in segments:
        arc = s.get('kind') in ('arc', 1)
        pts = _arc_points(s) if arc else [np.array(s['start']), np.array(s['end'])]
        pieces.append([np.array(p, float) for p in pts])
    chain = pieces.pop(0)
    while pieces:
        for i, p in enumerate(pieces):
            for rev in (False, True):
                q = p[::-1] if rev else p
                if np.linalg.norm(q[0] - chain[-1]) < 1e-6: chain += q[1:]; break
                if np.linalg.norm(q[-1] - chain[0]) < 1e-6: chain = q[:-1] + chain; break
            else: continue
            pieces.pop(i); break
        else: raise ValueError('Hoop segments do not form one chain')
    return chain


# Main hoop centreline. 'source' is the team's Main Roll Hoop Sketch exactly as supplied
# (4.0/3.75 in radii, a 0.38 deg line-line corner per side at z 626 mm); the team requires it
# from candidate 009d on. 'repair-002' is Codex's 3.25 in die repair that candidates up to 009c
# were built on (within 8.1 mm of the sketch). A recipe names its path in 'mainHoopPath';
# recipes without the key predate the choice and used the repair.
MAIN_HOOP_PATHS = ('source', 'repair-002')


def main_hoop_path(recipe=None):
    return (recipe or {}).get('mainHoopPath', 'repair-002')


def main_hoop_segments(path):
    if path == 'source':
        return json.loads((ROOT / 'design/constraint-ledger.json').read_text(encoding='utf-8-sig'))['hoops'][0]['worldSegments']
    if path == 'repair-002':
        return json.loads((ROOT / 'design/main-hoop-repair-002.json').read_text(encoding='utf-8-sig'))['segments']
    raise ValueError(f'Unknown main hoop path {path!r}; expected one of {MAIN_HOOP_PATHS}')


def hoops(main_hoop='repair-002'):
    front = json.loads((ROOT / 'design/constraint-ledger.json').read_text(encoding='utf-8-sig'))['hoops'][1]['worldSegments']
    return {'MainHoop_1': hoop_polyline(main_hoop_segments(main_hoop)), 'FrontHoop_1': hoop_polyline(front)}


HOOP_PROFILE = '1.000 x 0.095.SLDLFP'


# ---------------------------------------------------------------- model build
class Frame:
    def __init__(self):
        self.xyz, self.elems, self.names = [], [], {}
        self._buf = np.empty((256, 3))   # coordinates of xyz, for vectorised lookups

    def node(self, p, name=None, tol=None):
        """Index of the first node within tol of p (following merge aliases), else a new node."""
        tol = NODE_MERGE if tol is None else tol
        p = np.asarray(p, float); n = len(self.xyz)
        if n:
            hit = np.flatnonzero(np.sqrt(((self._buf[:n] - p) ** 2).sum(axis=1)) < tol)
            if hit.size:
                i = int(hit[0])
                if name: self.names.setdefault(name, i)
                return getattr(self, 'alias', {}).get(i, i)
        if n == len(self._buf): self._buf = np.vstack([self._buf, np.empty_like(self._buf)])
        self._buf[n] = p
        self.xyz.append(p)
        if name: self.names[name] = len(self.xyz) - 1
        return len(self.xyz) - 1


def build(recipe, sizes=None):
    """Centreline frame from a generator recipe. sizes: optional {member: profile} overrides."""
    sizes = sizes or {}
    lines = []  # (a, b, section, member name)
    for m in recipe['members']:
        lines.append((np.array(m['a']), np.array(m['b']), section(sizes.get(m['name'], m['profile'])), m['name']))
    hoop_sec = section(HOOP_PROFILE)
    for name, poly in hoops(main_hoop_path(recipe)).items():
        for a, b in zip(poly, poly[1:]):
            lines.append((a, b, hoop_sec, name))
    # Split points: every member end, plus probes at crossbar centres.
    points = [p for a, b, *_ in lines for p in (a, b)] + [np.array(p) for p in recipe['probes']]
    f = Frame()
    for n, p in recipe['nodes'].items(): f.node(p, n)
    pieces = []
    P = np.unique(np.array(points), axis=0)
    for a, b, sec, name in lines:
        v = b - a; L = np.linalg.norm(v); u = v / L
        t = (P - a) @ u
        off = np.linalg.norm(P - (a + t[:, None] * u), axis=1)
        # Touching joint: end lies within the two outer radii + 0.05 in clearance.
        on = (t > 1e-6) & (t < L - 1e-6) & (off <= sec['od'] + JOINT_CLEARANCE)
        ts = sorted({0.0, 1.0} | set((t[on] / L).tolist()))
        for t0, t1 in zip(ts, ts[1:]):
            pieces.append((a + t0 * v, a + t1 * v, sec, name))
    for a, b, sec, name in pieces:
        i, j = f.node(a), f.node(b)
        if i != j: f.elems.append((i, j, sec, name))
    # Offset joints: a tube end that touches another tube without sharing its node
    # (e.g. a suspension pickup 5 mm off the hoop centreline) is MERGED into the
    # host's nearest split node, as SolidWorks merges beam joints within its
    # clearance. Hosts are the hoops and base-recipe tubes (never optional 'Cand_'/'Add_'
    # tubes or X-brace pieces), so the merge set does not change with the optimizer's choices. Short
    # rigid links are avoided: they wreck the conditioning of the stiffness matrix.
    xyz = np.array(f.xyz)
    owners, nodes_of = {}, {}
    for i, j, s_, name in f.elems:
        for k in (i, j): owners.setdefault(k, set()).add(name)
        nodes_of.setdefault(name, set()).update((i, j))
    reach = 2 * 0.0127 + JOINT_CLEARANCE
    hoop_names = {'MainHoop_1', 'FrontHoop_1'}
    alias = {}
    # Host node table in host order (first minimum wins, as a scan over hosts would).
    # Add_ / XBrace: optimizer tubes under SolidWorks-safe names. X-brace pieces (optimizer XT_/XA_/XB_)
    # were hosts until 20 Oct: in the layout engine, which builds every candidate at once, an inactive
    # X-brace's crossing node then captured a nearby active crossing node (up to ~27 mm), kinking it,
    # and the engine scored 011b's tubes 4.7 % below this model. No recipe score changed.
    hosts = [h for h in nodes_of if not h.startswith(('Cand_', 'Add_', 'XT_', 'XA_', 'XB_', 'XBrace'))]
    host_id = {h: k for k, h in enumerate(hosts)}
    hn_node = np.array([k for h in hosts for k in nodes_of[h]], int)
    hn_host = np.array([host_id[h] for h in hosts for _ in nodes_of[h]], int)
    hn_xyz = xyz[hn_node]
    for a, b, sec, name in lines:
        if name in hoop_names: continue
        for end in (f.node(a), f.node(b)):
            excl = [host_id[h] for h in owners[end] | {name} if h in host_id]
            d = np.linalg.norm(hn_xyz - xyz[end], axis=1)
            ok = (d <= reach) & ~np.isin(hn_host, excl)
            if ok.any() and end not in alias:
                cand = np.flatnonzero(ok); alias[end] = int(hn_node[cand[np.argmin(d[cand])]])
    def root(k):
        seen = set()
        while k in alias and k not in seen: seen.add(k); k = alias[k]
        return k
    f.alias = {k: root(k) for k in alias}
    f.elems = [(root(i), root(j), s_, name) for i, j, s_, name in f.elems if root(i) != root(j)]
    f.xyz = xyz
    return f


# ---------------------------------------------------------------- FE solve
SHEAR_K = 0.5   # SolidWorks beam shear factors K1 = K2 ~ 0.50 (exported deck)


def shear_phi(L, s):
    """Timoshenko shear parameter 12 E I / (k G A L^2); 0 for A = 0."""
    return 12 * E * s['I'] / (SHEAR_K * G * s['A'] * L * L) if s['A'] > 0 else 0.0


def _k_local(L, s):
    A, I, J = s['A'], s['I'], s['J']
    k = np.zeros((12, 12))
    EA, GJ = E * A / L, G * J / L
    k[0, 0] = k[6, 6] = EA; k[0, 6] = k[6, 0] = -EA
    k[3, 3] = k[9, 9] = GJ; k[3, 9] = k[9, 3] = -GJ
    phi = shear_phi(L, s)
    for (v, r, sgn) in ((1, 5, 1), (2, 4, -1)):   # bending in y (about z) and z (about y), Timoshenko
        a, b = 12 * E * I / (L**3 * (1 + phi)), 6 * E * I / (L**2 * (1 + phi))
        c, d = (4 + phi) * E * I / (L * (1 + phi)), (2 - phi) * E * I / (L * (1 + phi))
        idx = [v, r, v + 6, r + 6]
        m = np.array([[a, sgn * b, -a, sgn * b], [sgn * b, c, -sgn * b, d],
                      [-a, -sgn * b, a, -sgn * b], [sgn * b, d, -sgn * b, c]])
        for p in range(4):
            for q in range(4): k[idx[p], idx[q]] += m[p, q]
    return k


def _rotation(a, b):
    x = (b - a) / np.linalg.norm(b - a)
    ref = np.array([0, 0, 1.0]) if abs(x[2]) < 0.9 else np.array([0, 1.0, 0])
    y = np.cross(ref, x); y /= np.linalg.norm(y); z = np.cross(x, y)
    R = np.vstack([x, y, z])
    T = np.zeros((12, 12))
    for i in range(4): T[3 * i:3 * i + 3, 3 * i:3 * i + 3] = R
    return T


def solve(f, recipe, load_scale=1.0):
    n = len(f.xyz); K = np.zeros((6 * n, 6 * n))
    for i, j, s, _ in f.elems:
        a, b = f.xyz[i], f.xyz[j]; T = _rotation(a, b)
        ke = T.T @ _k_local(np.linalg.norm(b - a), s) @ T
        dof = np.r_[6 * i:6 * i + 6, 6 * j:6 * j + 6]
        K[np.ix_(dof, dof)] += ke
    F = np.zeros(6 * n)
    loads = []
    for p in recipe['loadPoints']:
        k = f.node(p); fz = math.copysign(100 * LBF, p[1]) * load_scale
        F[6 * k + 2] += fz; loads.append((np.array(p), fz))
    fixed = {6 * f.node(p) + c for p in recipe['fixturePoints'] for c in range(3)}
    used = {k for i, j, *_ in f.elems for k in (i, j)}
    fixed |= {6 * k + c for k in range(n) if k not in used for c in range(6)}  # unused datum nodes
    fixed = sorted(fixed)
    free = np.setdiff1d(np.arange(6 * n), fixed)
    Kff = K[np.ix_(free, free)]
    try:
        u_free = np.linalg.solve(Kff, F[free])
    except np.linalg.LinAlgError:
        return None  # mechanism / disconnected
    if not np.all(np.isfinite(u_free)) or np.abs(u_free).max() > 1: return None
    u = np.zeros(6 * n); u[free] = u_free
    rx = [math.degrees(u[6 * f.node(p) + 3]) for p in recipe['probes']]
    torque_nm = sum(p[1] * fz for p, fz in loads)
    return {'rxDeg': rx, 'rxAvgDeg': float(np.mean(rx)), 'actualTorqueLbFt': torque_nm / LBFT, 'u': u}


def mass_kg(f, recipe=None):
    """Centreline mass, or with `recipe` the trimmed estimate: each tube end sits about one
    carrier radius (12.7 mm) inside the tube it is coped onto (matches SolidWorks within 0.3%)."""
    kg = float(sum(np.linalg.norm(f.xyz[j] - f.xyz[i]) * s['kgPerM'] for i, j, s, name in f.elems))
    if recipe:
        kg -= sum(min(2 * 0.0127, 0.5 * np.linalg.norm(np.subtract(m['b'], m['a']))) * section(m['profile'])['kgPerM']
                  for m in recipe['members'])
    return kg


def evaluate(recipe, sizes=None):
    f = build(recipe, sizes)
    r = solve(f, recipe)
    kg = mass_kg(f, recipe); lb = kg / 0.45359237
    if r is None: return {'valid': False, 'reason': 'mechanism or disconnected', 'weightLb': lb}
    tr = TRACKER_TORQUE_LBFT / abs(r['rxAvgDeg'])
    return {'valid': True, 'rxDeg': r['rxDeg'], 'rxAvgDeg': r['rxAvgDeg'], 'trLbFtPerDeg': tr,
            'trActualTorqueLbFtPerDeg': abs(r['actualTorqueLbFt'] / r['rxAvgDeg']), 'actualTorqueLbFt': r['actualTorqueLbFt'],
            'weightLb': lb, 'trPerLb': tr / lb, 'nodes': len(f.xyz), 'elements': len(f.elems)}


if __name__ == '__main__':
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else 'design/frame-005.json'
    rec = json.loads((ROOT / path).read_text(encoding='utf-8-sig'))
    print(json.dumps(evaluate(rec), indent=2))
