"""Ground-structure layout and tube-size search, scored like the team TR tracker.

How it works
1. Fixed points ("ground nodes"): every node of a base recipe (hoop attachment
   points, bulkhead corners, suspension pickups, cockpit side posts).
2. Required members: the rule-mandated structure of the base recipe (bulkheads,
   hoop braces and supports, side impact, suspension rails, battery protection,
   probe crossbars). Always present, at the rule size class or larger.
3. Candidate members: every other pair of ground nodes that is a sensible length
   and does not pass through a keep-out solid, a driver/cockpit template sweep, or
   the driver template. Each candidate is absent or present at an allowed size.
4. Score: TR / weight exactly like the tracker (545 lb-ft / mean probe RX / lb),
   from a 3D beam model (scripts/beam_model.py) assembled from precomputed element
   matrices, so one evaluation costs a few milliseconds.
5. Search: greedy best-improvement moves (add, remove, resize), then random
   perturb-and-reoptimise restarts. Crossing tubes and too-shallow joint angles
   are forbidden pairs.

The result is a recipe in the generator format (groups, members, weld-order trims)
for scripts/chassis_pipeline.py to build, trim and simulate in SolidWorks. The
SolidWorks study remains the score of record.

  python scripts/ground_structure.py --base design/frame-005.json --out design/frame-006.json
"""
from pathlib import Path
import argparse, itertools, json, math, random, re, sys, time
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from scipy.spatial import ConvexHull

sys.path.insert(0, str(Path(__file__).parent))
import beam_model as bm
import packaging as pkg
import photo_keepouts
import rule_structure as rs
from frame_geometry import make_trims, source_geometry, PROFILES

ROOT = bm.ROOT
# FSAE 2027 F.3.4.1 minimums: I mm4, A mm2, OD mm, wall mm
CLASS = rs.CLASS
RANK = {'C': 0, 'B': 1, 'A': 2}
# Profiles the native trimmer handles today (1.000 in round). scripts/native cutter
# generalisation extends this list (see --catalog extended).
CATALOG = {
    'round1': ['1.000 x 0.049.sldlfp', '1.000 x 0.058.sldlfp', '1.000 x 0.065.sldlfp',
               '1.000 x 0.083.sldlfp', '1.000 x 0.095.SLDLFP'],
}
CATALOG['extended'] = CATALOG['round1'] + ['1.375 x 0.049.SLDLFP', '1.000SQx 0.049.sldlfp', '1.000SQx 0.065.sldlfp']
# Round tubes the native trimmer copes (any OD), including Non-Structural sizes (F.3.3)
# that optional tubes may use; they never count toward a rule requirement.
CATALOG['round'] = CATALOG['round1'] + ['1.375 x 0.049.SLDLFP', '0.750 x 0.035.sldlfp', '0.750 x 0.049.sldlfp',
                                        '0.750 x 0.065.sldlfp', '1.000 x 0.035.sldlfp']
# Only sizes on VR3's "normally stocked" 4130N round tubing chart (VR3 guide p.22). 1.375 x 0.049
# is NOT stocked (1.375 comes in .035/.058/.065/...). Larger thin-wall ODs give the most bending
# and torsion stiffness per pound; .035 walls are Non-Structural (F.3.3) and count toward no rule.
CATALOG['vr3'] = ['0.750 x 0.035.sldlfp', '0.750 x 0.049.sldlfp', '0.750 x 0.065.sldlfp',
                  '1.000 x 0.035.sldlfp', '1.000 x 0.049.sldlfp', '1.000 x 0.058.sldlfp', '1.000 x 0.065.sldlfp',
                  '1.000 x 0.083.sldlfp', '1.000 x 0.095.SLDLFP', '1.125 x 0.049.sldlfp', '1.250 x 0.035.sldlfp',
                  '1.250 x 0.049.sldlfp', '1.375 x 0.058.sldlfp', '1.500 x 0.035.sldlfp', '1.500 x 0.049.sldlfp']
# The engine also carries the unstocked 1.375 x 0.049 as a phase-1 search aid only
# (layout_engine.two_phase_search removes it before any result is produced).
CATALOG['vr3-search'] = CATALOG['vr3'] + ['1.375 x 0.049.SLDLFP']

# Rule-required structure and its minimum size class (design/research/rule-constraints.md,
# section 6). Phase A keeps Codex's rule-panel topology (it passes the rule audit) as the
# required core; everything the rules do not ask for is optional.
REQUIRED_ROLES = {
    'front_bulkhead': 'B', 'front_bulkhead_diagonal': 'C',                  # F.6.1, F.8.4.3
    'bulkhead_lower_support': 'C', 'bulkhead_upper_support': 'C', 'bulkhead_support_triangulation': 'C',  # F.6.2
    'front_hoop_brace': 'B',                                                # F.6.3
    'main_brace_and_lower_support': 'B',                                    # F.5.9, F.6.6
    'main_brace_upper_support': 'B',                                        # F.11.2 battery upper protection
    'side_lower': 'B', 'side_upper': 'B', 'side_diagonal': 'B', 'side_triangulation': 'B',  # F.6.4
    'rear_bulkhead': 'C', 'rear_impact_upright': 'C',                       # F.11.3.1.b (battery 466 mm away)
    'battery_protection': 'B',
    'powertrain_protection': 'C',                                           # F.11.1.4, F.5.16, F.3.2.1.n                                              # F.11.2
    'rear_triangulation': 'C',                                              # F.11.3.4 load paths (EV)
}
# Named groups: the two probe bars (procedure RX1/RX2) and the rear rails, which carry
# the F.11.3.4 load paths. Front rails, base crossbar, cockpit floor and the rear lower
# miter support are not required by any rule or by the task.
REQUIRED_GROUPS = {'Weld_FrontProbeFore': 'C', 'Weld_FrontBase': 'C',
                   'Weld_RearSuspensionUpperL': 'C', 'Weld_RearSuspensionUpperR': 'C',
                   'Weld_RearSuspensionLowerL': 'C', 'Weld_RearSuspensionLowerR': 'C'}
# Free-skeleton mode (Problem(skeleton='free') or GS_SKELETON=free): Codex's side impact panels
# (F.6.4) and front bulkhead supports (F.6.2.3) stop being fixed. Their tubes become optional like any
# candidate, every other node pair may replace them, and a state is valid only while the generic,
# name-free predicates in scripts/rule_structure.py find a complete structure on each side. These
# roles only say which base tubes are released; acceptance never looks at roles or names.
SKELETON_ROLES = {'side_lower', 'side_upper', 'side_diagonal', 'side_triangulation',
                  'bulkhead_lower_support', 'bulkhead_upper_support', 'bulkhead_support_triangulation'}
SKELETON_MODE = __import__('os').environ.get('GS_SKELETON', 'fixed')
# Hulls the rule-required frame must clear (param_search precheck): every keep-out, template sweep and
# photo zone except the supplied +10 mm internal templates (conservative hulls, rule-constraints G24).
RULE_HULL = lambda name: not name.startswith('SuppliedOversize') or name == 'SuppliedOversizeOpening'
PROBE_BARS = ('FrontProbeFore', 'FrontBase')   # nothing may land on their interiors
# Producibility preferences (not rules): see rule-constraints.md G20.
MIN_LEN, MAX_LEN, MIN_ANGLE_DEG = 0.12, 1.1, 20.0
# Joints between two of Codex's base tubes were coped natively (candidate-005, all trims
# applied and the tube-intersection audit passed) down to 16 deg; they keep that floor.
BASE_MIN_ANGLE_DEG = 15.0
CARRIER_R = 0.0127      # tube ends sit inside the 1.0 in carrier they are coped onto
HALFSHAFT_R = 0.028     # rear halfshaft corridor radius: ~25-30 mm shaft with ~10 mm motion margin
KINK_PLANE_DEG = 30.0   # F.5.2.3.c support within 30 deg of the member plane
ATTACH_DENSE = __import__('os').environ.get('GS_ATTACH') == 'dense'   # experiment switch; default off


def rule_class(name, role):
    """Minimum size class for a base group, or None if the group is optional."""
    return REQUIRED_GROUPS.get(name) or REQUIRED_ROLES.get(role)


def meets(profile, cls):
    s = bm.section(profile); i, a, od, w = CLASS[cls]
    return s['I'] * 1e12 >= i and s['A'] * 1e6 >= a and s['od'] * 1e3 >= od - 1e-9 and s['wall'] * 1e3 >= w - 1e-9


# ------------------------------------------------------------------ forbidden volumes
class Forbidden:
    """Keep-out solids + driver/cockpit template sweeps as convex hulls. accumulator: a
    packaging.accumulator_box() that replaces the starter battery solid (scenario sizes)."""
    def __init__(self, accumulator=None):
        self.hulls = []
        _, refs = source_geometry()
        for b in refs:
            v = np.array(b['verticesM'])
            if accumulator and b['name'] == accumulator['name']: v = pkg.box_vertices(accumulator)
            self.hulls.append((b['name'], ConvexHull(v).equations))
        checks = json.loads((ROOT / 'design/driver-checks-005.json').read_text(encoding='utf-8-sig'))
        for e in checks['sweptEnvelopes']:
            poly = np.array(e['polygon']); d = np.array(e['direction']) * e['depthM']
            pts = np.vstack([poly, poly + d])
            if e['name'] == 'RuleDriverOutline':  # planar template: give it the tube's reach either side
                pts = np.vstack([poly + [0, -.02, 0], poly + [0, .02, 0]])
            self.hulls.append((e['name'], ConvexHull(pts).equations))
        # Packaging zones estimated from photos of car 61 (scripts/photo_keepouts.py).
        self.MARGIN = dict(Forbidden.MARGIN)
        for z in photo_keepouts.load():
            self.hulls.append(('Photo:' + z['name'], ConvexHull(np.array(z['verticesM'])).equations))
            self.MARGIN['Photo:' + z['name']] = z.get('marginM', 0.0)

    # F.11.4.2: the battery *should* have 25 mm clearance to impact structure. New tubes keep
    # it; the rule-required battery protection tubes are placed by the generator.
    MARGIN = {'Boss-Extrude1': 0.025, 'Boss-Extrude2': 0.025}

    def add_halfshaft_corridors(self, x0, z0, radius=HALFSHAFT_R, y_in=0.150, y_out=0.260, n=32):
        """Rear halfshaft path (user: rear motor + halfshafts; axle line from packaging.axle_line):
        a cylinder along Y through each rear frame side."""
        ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
        ring = np.stack([x0 + radius * np.cos(ang), np.zeros(n), z0 + radius * np.sin(ang)], axis=1)
        for side, sgn in (('L', 1), ('R', -1)):
            pts = np.vstack([ring + [0, sgn * y_in, 0], ring + [0, sgn * y_out, 0]])
            self.hulls.append((f'HalfshaftCorridor{side}', ConvexHull(pts).equations))

    def clearance(self, a, b, only=None):   # exact (seg_hull_min): sampling missed grazing contacts (frame-012 at 5 mm; 0.06 mm at 1 mm)
        """Smallest signed distance (minus each solid's margin) from the segment ab, ends
        excluded, to any keep-out hull; and that hull's name. Negative = inside."""
        best = (np.inf, None)
        for name, eq in self.hulls:
            if only and not name.startswith(only): continue
            d = photo_keepouts.seg_hull_min(a, b, eq) - self.MARGIN.get(name, 0.0)
            if d < best[0]: best = (d, name)
        return best

    def raw_clearance(self, a, b, keep=lambda name: True):
        """Smallest signed centreline distance (no margins) from segment ab, ends excluded, to the
        kept hulls: the test param_search.precheck applies to rule-required tubes."""
        return min((photo_keepouts.seg_hull_min(a, b, eq) for name, eq in self.hulls if keep(name)), default=np.inf)

    def hit(self, a, b, radius=0.0127):
        for name, eq in self.hulls:   # ends excluded: they sit on existing structure
            if photo_keepouts.seg_hull_min(a, b, eq) < radius + self.MARGIN.get(name, 0.0): return name
        return None


def seg_dist(p1, q1, p2, q2):
    """Minimum distance between segments p1q1 and p2q2 (either may be a point)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    if a < 1e-18 and e < 1e-18: return float(np.linalg.norm(r))
    if a < 1e-18: s, t = 0.0, np.clip(f / e, 0, 1)
    else:
        c = d1 @ r
        if e < 1e-18: t, s = 0.0, np.clip(-c / a, 0, 1)
        else:
            b = d1 @ d2; den = a * e - b * b
            s = np.clip((b * f - c * e) / den, 0, 1) if den > 1e-18 else 0.0
            t = (b * s + f) / e
            if t < 0: t, s = 0.0, np.clip(-c / a, 0, 1)
            elif t > 1: t, s = 1.0, np.clip((b - c) / a, 0, 1)
    return float(np.linalg.norm((p1 + s * d1) - (p2 + t * d2)))


def seg_seg_dist(P1, d1, P2, d2):
    """Vectorised minimum distance between segments P1+s*d1 and P2+t*d2 (s, t in [0, 1])."""
    r = P1 - P2
    a, e = np.einsum('ij,ij->i', d1, d1), np.einsum('ij,ij->i', d2, d2)
    b, c, f = np.einsum('ij,ij->i', d1, d2), np.einsum('ij,ij->i', d1, r), np.einsum('ij,ij->i', d2, r)
    den = a * e - b * b; ok = den > 1e-18
    s = np.where(ok, np.clip((b * f - c * e) / np.where(ok, den, 1), 0, 1), 0.0)
    t = (b * s + f) / e
    s = np.where(t < 0, np.clip(-c / a, 0, 1), np.where(t > 1, np.clip((b - c) / a, 0, 1), s))
    t = np.clip(t, 0, 1)
    return np.linalg.norm((P1 + s[:, None] * d1) - (P2 + t[:, None] * d2), axis=1)


def segment_conflicts(P1, Q1, A1, B1, P2, Q2, A2, B2, min_angle=MIN_ANGLE_DEG):
    """Vectorised Problem._conflict for arrays of segment pairs (node ids A/B, ends P/Q);
    min_angle may be a per-pair array."""
    reach = 0.0254 + bm.JOINT_CLEARANCE
    d1, d2 = Q1 - P1, Q2 - P2
    L1, L2 = np.linalg.norm(d1, axis=1), np.linalg.norm(d2, axis=1)
    uv = np.einsum('ij,ij->i', d1, d2) / (L1 * L2)
    near = lambda X, Y: np.linalg.norm(X - Y, axis=1) < 1e-6
    sAA, sAB = (A1 == A2) | near(P1, P2), (A1 == B2) | near(P1, Q2)
    sBA, sBB = (B1 == A2) | near(Q1, P2), (B1 == B2) | near(Q1, Q2)
    shared = sAA | sAB | sBA | sBB
    # directions leaving the shared node
    s1 = np.where(sAA | sAB, 1.0, -1.0); s2 = np.where(sAA | sBA, 1.0, -1.0)
    shallow_joint = np.degrees(np.arccos(np.clip(uv * s1 * s2, -1, 1))) < min_angle
    line_angle = np.degrees(np.arccos(np.clip(np.abs(uv), 0, 1)))
    def pt_seg(X, P, d, L):
        t = np.clip(np.einsum('ij,ij->i', X - P, d) / (L * L), 0, 1)
        return np.linalg.norm(X - (P + t[:, None] * d), axis=1)
    tjoint = ((pt_seg(P1, P2, d2, L2) <= reach) | (pt_seg(Q1, P2, d2, L2) <= reach) |
              (pt_seg(P2, P1, d1, L1) <= reach) | (pt_seg(Q2, P1, d1, L1) <= reach))
    crossing = seg_seg_dist(P1, d1, P2, d2) < 0.0254 + 0.002
    return np.where(shared, shallow_joint, np.where(tjoint, line_angle < min_angle, crossing))


# ------------------------------------------------------------------ problem
def rear_apex(nodes, axle, grid=np.arange(0.300, 0.9025, 0.005)):
    """Apex of the rear-bay A-frame (both lower rear pickups to one point of the upper rail) that
    keeps both legs farthest from the halfshaft axis, in the side plane. Returns (t, centreline
    distance in m); t is the fraction along RearSuspensionUpper from RAU to RFU (0.5 % steps)."""
    P = {k: np.array(nodes[k + 'L'])[[0, 2]] for k in ('RA0', 'RAU', 'RF0', 'RFU')}
    c = np.array([axle['xM'], axle['zM']])
    def dist(a, b):
        d = b - a; t = np.clip((c - a) @ d / (d @ d), 0, 1); return float(np.linalg.norm(c - (a + t * d)))
    best = max((min(dist(P['RA0'], A), dist(P['RF0'], A)), round(float(t), 3))
               for t in grid for A in [P['RAU'] + t * (P['RFU'] - P['RAU'])])
    return best[1], best[0]


def apex_label(t):
    """Attachment-point suffix: '50' for the midpoint, '74.5' for t = 0.745."""
    return f'{round(t * 1000) / 10:g}'


class Problem:
    def __init__(self, base, catalog, halfshafts=True, packaging=None, skeleton=None):
        """packaging: {'accumulator': packaging.accumulator_box(...) or None,
                       'axle': {'frac': f or None, 'zM': z or None} or None}; None keeps the
        starter battery solid and the axle midway between the rear pickups (009c inputs).
        skeleton: 'fixed' (Codex's side and bulkhead-support tubes required) or 'free' (see
        SKELETON_ROLES); default GS_SKELETON or 'fixed'."""
        self.base = base; self.catalog = catalog
        self.skeleton = skeleton or SKELETON_MODE
        if self.skeleton not in ('fixed', 'free'): raise ValueError(f'skeleton must be fixed or free, not {self.skeleton!r}')
        self.free = self.skeleton == 'free'
        pk = packaging or {}
        self.accumulator = pk.get('accumulator')
        self.axle = pkg.axle_line(base['nodes'], **pk['axle']) if halfshafts and pk.get('axle') else None
        # Rear-bay A-frame apex on the upper rail: its midpoint for the 009c axle estimate,
        # otherwise the point that keeps both legs farthest from the moved axle.
        self.apex_t, self.apex_dist = rear_apex(base['nodes'], self.axle) if self.axle else (1 / 2, None)
        self.nodes = {k: np.array(v) for k, v in base['nodes'].items()}
        self.hoop_nodes = {'MainHoop_1': [n for n in base['nodes'] if n.startswith('MH')],
                           'FrontHoop_1': [n for n in base['nodes'] if n.startswith('FH')]}
        # One canonical name per location (the recipe aliases some points, e.g. RAUL = RB3).
        self.canon = {}
        seen = {}
        for n, p in self.nodes.items():
            key = tuple(np.rint(p * 1e6).astype(int))
            self.canon[n] = seen.setdefault(key, n)
        self.groups = []
        for g in base['groups']:
            role = g['members'][0]['role']; cls = rule_class(g['name'], role)
            skel = self.free and role in SKELETON_ROLES   # released: optional, judged by rule_structure
            self.groups.append({'name': g['name'], 'role': role, 'cls': None if skel else cls, 'required': cls is not None and not skel,
                                'skeleton': skel,
                                'members': [(m['name'], self.canon[m['nodes'][0]], self.canon[m['nodes'][1]]) for m in g['members']],
                                'base': True, 'baseProfile': g['profile']})
        # Extra attachment points along existing tubes (T-joints): thirds and midpoints
        # of long members give the search places to land diagonals. None on the probe
        # bars: a tube landing there would change what RX measures.
        self.on_member = {}
        self.member_group = {m['name']: gi for gi, g in enumerate(base['groups']) for m in g['members']}
        for g in base['groups']:
            for m in g['members']:
                if m['name'] in PROBE_BARS: continue
                a, b = np.array(m['a']), np.array(m['b']); L = np.linalg.norm(b - a)
                ts = (1 / 3, 1 / 2, 2 / 3) if L > 0.6 else (1 / 2,) if L > 0.3 else ()
                if ATTACH_DENSE:   # finer attachment grid (GS_ATTACH=dense): quarters, plus thirds on 0.3-0.6 m tubes
                    ts = (1 / 4, 1 / 3, 1 / 2, 2 / 3, 3 / 4) if L > 0.6 else (1 / 3, 1 / 2, 2 / 3) if L > 0.3 else (1 / 2,) if L > 0.2 else ()
                apex = halfshafts and m['name'] in ('RearSuspensionUpperL', 'RearSuspensionUpperR')
                if apex: ts = (self.apex_t,)  # A-frame apex
                for t in ts:
                    name = f"{m['name']}@{apex_label(t) if apex else round(t * 100)}"
                    self.nodes[name] = a + t * (b - a); self.on_member[name] = m['name']; self.canon[name] = name
        self.r_min = min(bm.section(p)['od'] for p in catalog) / 2
        self.forbidden = Forbidden(self.accumulator)
        # Rear halfshafts: axle line estimated midway between the rear suspension pickups,
        # or the scenario axle line (packaging.axle_line).
        rear = [self.nodes[n] for n in ('RA0L', 'RAUL', 'RF0L', 'RFUL') if n in self.nodes]
        self.halfshaft = None
        if self.axle:
            x0, z0 = self.axle['xM'], self.axle['zM']
            self.forbidden.add_halfshaft_corridors(x0, z0); self.halfshaft = {'xM': x0, 'zM': z0, 'radiusM': HALFSHAFT_R}
        elif len(rear) == 4 and halfshafts:
            x0, z0 = float(np.mean([q[0] for q in rear])), float(np.mean([q[2] for q in rear]))
            self.forbidden.add_halfshaft_corridors(x0, z0); self.halfshaft = {'xM': x0, 'zM': z0, 'radiusM': HALFSHAFT_R}
        self.blocked = []   # base tubes crossing a halfshaft corridor: removed, replaced below
        self.photo_conflicts = []
        for g in self.groups:
            c = min(self.forbidden.clearance(self.nodes[a], self.nodes[b], only='Halfshaft')[0] for _, a, b in g['members'])
            if c < CARRIER_R:
                g.update(required=False, cls=None, blocked=True); self.blocked.append(g['name']); continue
            # Upsizing a base tube must keep its surface out of every keep-out and template;
            # its own (natively audited) size is always allowed.
            clear = min(self.forbidden.clearance(self.nodes[a], self.nodes[b])[0] for _, a, b in g['members'])
            g['maxR'] = max(clear, bm.section(g['baseProfile'])['od'] / 2)
            # Released skeleton tubes skip the param_search precheck, so they keep the own-size
            # exemption only where that check would have passed them.
            if g['skeleton'] and min(self.forbidden.raw_clearance(self.nodes[a], self.nodes[b], self._rule_hull)
                                     for _, a, b in g['members']) < CARRIER_R:
                g['maxR'] = clear
            # Photo packaging zones are estimates: they block new candidate tubes, but a base tube
            # inside one is kept and reported for a packaging decision, never silently removed.
            pc, pz = min(self.forbidden.clearance(self.nodes[a], self.nodes[b], only='Photo:') for _, a, b in g['members'])
            if pc < bm.section(g['baseProfile'])['od'] / 2:
                self.photo_conflicts.append({'group': g['name'], 'zone': pz, 'required': g['required'],
                                             'clearanceMm': round((pc - bm.section(g['baseProfile'])['od'] / 2) * 1e3, 1)})
        self.hoop_polys = list(bm.hoops(bm.main_hoop_path(base)).values())
        self.rejected = {}
        self.dropped_from_base = []
        self._candidates()
        if self.blocked: self._rear_bay_frames()
        self._model()
        self._symmetry()
        self._pickups()
        self._joints_and_kinks()
        if self.free: self._panels_setup()

    @staticmethod
    def _rule_hull(name):
        return RULE_HULL(name) and not name.startswith('Halfshaft')

    def seg(self, g):
        return [(self.nodes[a], self.nodes[b]) for _, a, b in g['members']]

    def _lines_through(self, n):
        """Base members whose centreline contains node n (as an end or an attachment point)."""
        if n in self.on_member: return {self.on_member[n]}
        return {m for g in self.groups if g['base'] for m, a, b in g['members'] if n in (a, b)}

    def _candidates(self):
        existing = {frozenset((a, b)) for g in self.groups for _, a, b in g['members']}
        names = sorted({self.canon[n] for n in self.nodes if not n.startswith('MHH')})  # MHH: unused datum
        # Fixed obstacles: required tubes and roll-hoop segments (matched by position only).
        obst = [pq for g in self.groups if g['required'] for pq in self.seg(g)]
        obst += [(p, q) for poly in self.hoop_polys for p, q in zip(poly, poly[1:])]
        OP, OQ = np.array([o[0] for o in obst]), np.array([o[1] for o in obst])
        no = len(obst); OA, OB = np.full(no, -1), np.full(no, -2)
        for a, b in itertools.combinations(names, 2):
            if frozenset((a, b)) in existing: continue
            pa, pb = self.nodes[a], self.nodes[b]; L = np.linalg.norm(pb - pa)
            if not MIN_LEN <= L <= MAX_LEN: continue
            if (a[:2] == b[:2] and a[:2] in ('MH', 'FH')): continue  # would duplicate the hoop
            # Both ends on one base tube: the candidate would lie along that tube.
            if (a in self.on_member or b in self.on_member) and self._lines_through(a) & self._lines_through(b): continue
            clear, why = self.forbidden.clearance(pa, pb)
            if clear < self.r_min: self.rejected[f'{a}-{b}'] = why; continue
            # Must not pass through or run along a required tube or a roll hoop.
            if segment_conflicts(np.broadcast_to(pa, OP.shape), np.broadcast_to(pb, OP.shape), np.zeros(no, int),
                                 np.ones(no, int), OP, OQ, OA, OB).any():
                self.rejected[f'{a}-{b}'] = 'crosses required tube or hoop'; continue
            self.groups.append({'name': f'Weld_Cand_{a}_{b}', 'role': 'candidate', 'cls': None, 'required': False,
                                'members': [(f'Cand_{a}_{b}', a, b)], 'base': False, 'baseProfile': None, 'maxR': clear})
        self._x_braces()
        # A tube ending at an attachment point needs that host tube (no mid-air joints).
        self.depends = {gi: {self.member_group[self.on_member[n]] for _, a, b in g['members'] for n in (a, b) if n in self.on_member}
                        for gi, g in enumerate(self.groups)}
        self.dependents = {gi: set() for gi in range(len(self.groups))}
        for gi, hosts in self.depends.items():
            for h in hosts: self.dependents[h].add(gi)
        # Pairwise conflicts (crossing tubes, shallow joints) between every pair of tube
        # segments where at least one group is optional: bounding-box prefilter, then the
        # _conflict rules evaluated for all candidate pairs at once.
        seg_g, P, Q, A, B, nid = [], [], [], [], [], {}
        for gi, g in enumerate(self.groups):
            for _, a, b in g['members']:
                seg_g.append(gi); P.append(self.nodes[a]); Q.append(self.nodes[b])
                A.append(nid.setdefault(a, len(nid))); B.append(nid.setdefault(b, len(nid)))
        seg_g, P, Q, A, B = np.array(seg_g), np.array(P), np.array(Q), np.array(A), np.array(B)
        # Pairs of tubes required in the fixed mode are never checked against each other (released
        # skeleton tubes included: they coexist in Codex's natively coped base).
        opt_seg = np.array([not (self.groups[g]['required'] or self.groups[g].get('skeleton')) for g in seg_g])
        lo, hi = np.minimum(P, Q) - 0.03, np.maximum(P, Q) + 0.03
        I, J = [], []
        for i in range(len(P)):
            js = np.nonzero(np.all((lo[i + 1:] <= hi[i]) & (hi[i + 1:] >= lo[i]), axis=1))[0] + i + 1
            js = js[(seg_g[js] != seg_g[i]) & (opt_seg[js] | opt_seg[i])]
            I.append(np.full(len(js), i)); J.append(js)
        I, J = np.concatenate(I), np.concatenate(J)
        base_seg = np.array([self.groups[g]['base'] for g in seg_g])
        floor = np.where(base_seg[I] & base_seg[J], BASE_MIN_ANGLE_DEG, MIN_ANGLE_DEG)
        hit = segment_conflicts(P[I], Q[I], A[I], B[I], P[J], Q[J], A[J], B[J], floor)
        self.conflicts = {gi: set() for gi in range(len(self.groups))}
        for i, j in zip(I[hit], J[hit]):
            gi, gj = int(seg_g[i]), int(seg_g[j]); self.conflicts[gi].add(gj); self.conflicts[gj].add(gi)

    def _rear_bay_frames(self):
        """A rear side diagonal across the halfshaft path is replaced by an A-frame: both lower
        rear pickups to the upper rail's midpoint. The bay stays triangulated (F.11.3.4 load
        paths, Size C) and the shaft passes through the triangle's interior."""
        apex = apex_label(self.apex_t)
        for side in ('L', 'R'):
            legs = [f'Weld_Cand_{a}_RearSuspensionUpper{side}@{apex}' for a in (f'RA0{side}', f'RF0{side}')]
            idx = {g['name']: gi for gi, g in enumerate(self.groups)}
            missing = [n for n in legs if n not in idx]
            if missing:
                why = '' if self.apex_dist is None else (
                    f'; best apex @{apex} puts the legs {self.apex_dist * 1000:.1f} mm from the axle, a Size C '
                    f'1.000 in leg needs {(HALFSHAFT_R + CARRIER_R) * 1000:.1f} mm')
                raise RuntimeError(f'Rear bay A-frame not available (keep-outs/crossings): {missing}{why}')
            for n in legs:
                g = self.groups[idx[n]]; g.update(role='rear_triangulation', cls='C', required=True)
                # Size C needs a 1.000 in tube; its surface must clear the corridor (and keep-outs).
                if g['maxR'] < CARRIER_R:
                    raise RuntimeError(f'Rear bay A-frame leg {n} clears the halfshaft corridor by only '
                                       f'{g["maxR"] * 1000:.1f} mm at its centreline; a Size C 1.000 in tube needs '
                                       f'{CARRIER_R * 1000:.1f} mm (best apex @{apex})')

    def _x_braces(self, max_gap=0.002, min_angle=35.0, t_lo=0.2, t_hi=0.8):
        """X-braces: two candidate diagonals that truly cross become one group - a through
        tube plus the other diagonal split at the crossing node and coped onto it (VR3
        layering; F.8.4.3 and F.11.3.5 name X braces). Crossing singles stay conflicts."""
        singles = [(gi, g['members'][0]) for gi, g in enumerate(self.groups) if not g['base'] and len(g['members']) == 1]
        P = np.array([self.nodes[m[1]] for _, m in singles]); Q = np.array([self.nodes[m[2]] for _, m in singles])
        lo, hi = np.minimum(P, Q) - max_gap, np.maximum(P, Q) + max_gap
        added = 0
        for i, (gi, (n1, a1, b1)) in enumerate(singles):
            near = np.where(np.all((lo <= hi[i]) & (hi >= lo[i]), axis=1))[0]
            for j in near[near > i]:
                gj, (n2, a2, b2) = singles[j]
                if {a1, b1} & {a2, b2}: continue
                p1, q1, p2, q2 = P[i], Q[i], P[j], Q[j]
                d1, d2, r = q1 - p1, q2 - p2, p1 - p2
                A, B, C, D, E = d1 @ d1, d1 @ d2, d2 @ d2, d1 @ r, d2 @ r
                den = A * C - B * B
                if den < 1e-12: continue
                s, t = (B * E - C * D) / den, (A * E - B * D) / den
                if not (t_lo < s < t_hi and t_lo < t < t_hi): continue
                x1, x2 = p1 + s * d1, p2 + t * d2
                if np.linalg.norm(x1 - x2) > max_gap: continue
                ang = math.degrees(math.acos(min(1, abs(d1 @ d2) / math.sqrt(A * C))))
                if ang < min_angle: continue
                xn = f'X[{n1}|{n2}]'; self.nodes[xn] = x1; self.canon[xn] = xn
                self.groups.append({'name': f'Weld_X_{n1}__{n2}', 'role': 'x_brace', 'cls': None, 'required': False,
                                    'members': [(f'XT_{n1}__{n2}', a1, b1), (f'XA_{n1}__{n2}', a2, xn), (f'XB_{n1}__{n2}', xn, b2)],
                                    'base': False, 'baseProfile': None})
                added += 1
        self.x_count = added

    def _conflict(self, s1, s2):
        p1, q1, a1, b1 = s1; p2, q2, a2, b2 = s2
        shared = {a1, b1} & {a2, b2} - {None}
        if not shared:
            # positional sharing (same coordinates under another name)
            for x, y in ((p1, p2), (p1, q2), (q1, p2), (q1, q2)):
                if np.linalg.norm(x - y) < 1e-6: shared = {'pos'}
        if shared:
            # shallow angle at a shared joint cannot be coped/welded
            if a1 in (a2, b2) or 'pos' in shared:
                o = p1 if (a1 in (a2, b2) or np.linalg.norm(p1 - p2) < 1e-6 or np.linalg.norm(p1 - q2) < 1e-6) else q1
            else:
                o = q1
            u = (q1 - p1) if np.allclose(o, p1) else (p1 - q1)
            o2 = p2 if np.linalg.norm(p2 - o) < 1e-6 else q2
            v = (q2 - p2) if np.allclose(o2, p2) else (p2 - q2)
            ang = math.degrees(math.acos(np.clip(u @ v / (np.linalg.norm(u) * np.linalg.norm(v)), -1, 1)))
            return ang < MIN_ANGLE_DEG
        # T-joint: an end touching another tube's side is a legitimate joint if
        # the tubes are not near-parallel (a shallow saddle cannot be coped).
        u, v = (q1 - p1) / np.linalg.norm(q1 - p1), (q2 - p2) / np.linalg.norm(q2 - p2)
        line_angle = math.degrees(math.acos(min(1.0, abs(u @ v))))
        for x in (p1, q1, p2, q2):
            other = (p2, q2) if x is p1 or x is q1 else (p1, q1)
            if seg_dist(x, x, *other) <= 0.0254 + bm.JOINT_CLEARANCE: return line_angle < MIN_ANGLE_DEG
        return seg_dist(p1, q1, p2, q2) < 0.0254 + 0.002

    # -------------------------------------------------------------- fast model
    def _model(self):
        """All candidate geometry split once; element matrices kept as A/I/J parts."""
        rec = dict(self.base); rec['members'] = []
        for g in self.groups:
            for name, a, b in g['members']:
                rec['members'].append({'name': name, 'a': self.nodes[a].tolist(), 'b': self.nodes[b].tolist(),
                                       'profile': '1.000 x 0.065.sldlfp'})
        f = bm.build(rec)
        owner = {}
        for gi, g in enumerate(self.groups):
            for name, *_ in g['members']: owner[name] = gi
        self.xyz = f.xyz; n = len(f.xyz); self.ndof = 6 * n
        self.el_owner, self.el_len, rows, cols, parts, self.el_link = [], [], [], [], [], []
        self.hoop_k = None
        hoop_s = bm.section(bm.HOOP_PROFILE)
        Kfix = np.zeros(0)
        fixed_rows, fixed_cols, fixed_vals = [], [], []
        always = set()
        self.el_owner2 = []  # second owner for offset-joint links (-1: none / always present)
        link_s = bm.section(bm.HOOP_PROFILE)
        for i, j, s, name in f.elems:
            a, b = f.xyz[i], f.xyz[j]; L = np.linalg.norm(b - a); T = bm._rotation(a, b)
            kA = T.T @ bm._k_local(L, {'A': 1, 'I': 0, 'J': 0}) @ T
            kI = T.T @ bm._k_local(L, {'A': 0, 'I': 1, 'J': 0}) @ T
            kJ = T.T @ bm._k_local(L, {'A': 0, 'I': 0, 'J': 1}) @ T
            dof = np.r_[6 * i:6 * i + 6, 6 * j:6 * j + 6]
            rr, cc = np.meshgrid(dof, dof, indexing='ij')
            if name.startswith('link|'):
                _, m, h = name.split('|')
                o1, o2 = owner.get(m, -1), owner.get(h, -1)
                if o1 < 0: o1, o2 = o2, -1
                if o1 >= 0:
                    self.el_owner.append(o1); self.el_owner2.append(o2); self.el_len.append(0.0); self.el_link.append(True)
                    rows.append(rr.ravel()); cols.append(cc.ravel())
                    parts.append(((link_s['A'] * kA).ravel(), (link_s['I'] * kI).ravel(), (link_s['J'] * kJ).ravel()))
                    continue
            if name in owner:
                self.el_owner.append(owner[name]); self.el_owner2.append(-1); self.el_len.append(L); self.el_link.append(False)
                rows.append(rr.ravel()); cols.append(cc.ravel()); parts.append((kA.ravel(), kI.ravel(), kJ.ravel()))
            else:  # hoops (and links between hoops): always present
                always.update((i, j))
                k = hoop_s['A'] * kA + hoop_s['I'] * kI + hoop_s['J'] * kJ
                fixed_rows.append(rr.ravel()); fixed_cols.append(cc.ravel()); fixed_vals.append(k.ravel())
        self.el_owner = np.array(self.el_owner); self.el_len = np.array(self.el_len)
        self.el_owner2 = np.array(self.el_owner2); self.el_link = np.array(self.el_link)
        self.rows = np.concatenate(rows); self.cols = np.concatenate(cols)
        self.kA = np.concatenate([p[0] for p in parts]); self.kI = np.concatenate([p[1] for p in parts]); self.kJ = np.concatenate([p[2] for p in parts])
        self.el_conn = [(i, j) for i, j, s, name in f.elems if name in owner or (name.startswith('link|') and (owner.get(name.split('|')[1], -1) >= 0 or owner.get(name.split('|')[2], -1) >= 0))]
        self.fixed_K = sp.coo_matrix((np.concatenate(fixed_vals), (np.concatenate(fixed_rows), np.concatenate(fixed_cols))),
                                     shape=(self.ndof, self.ndof)).tocsr()
        self.hoop_mass = sum(np.linalg.norm(f.xyz[j] - f.xyz[i]) * hoop_s['kgPerM'] for i, j, s, name in f.elems if name in ('MainHoop_1', 'FrontHoop_1'))
        self.hoop_nodeset = always
        self.F = np.zeros(self.ndof)
        for p in self.base['loadPoints']:
            self.F[6 * f.node(p) + 2] += math.copysign(100 * bm.LBF, p[1])
        self.fix_dofs = {6 * f.node(p) + c for p in self.base['fixturePoints'] for c in range(3)}
        self.probe_nodes = [f.node(p) for p in self.base['probes']]
        self.sections = {p: bm.section(p) for p in self.catalog}
        self.allowed = []
        for g in self.groups:  # optional tubes may use any catalog size, incl. Non-Structural
            if g.get('blocked'): self.allowed.append([]); continue
            r_max = g.get('maxR', np.inf)   # keep-out clearance of the centreline (candidates)
            ok = [p for p in self.catalog if (g['cls'] is None or meets(p, g['cls'])) and self.sections[p]['od'] / 2 <= r_max + 1e-9]
            self.allowed.append(ok)

    def evaluate(self, state):
        """state: list of profile name or None per group -> metrics dict."""
        A = np.zeros(len(self.groups)); I = np.zeros(len(self.groups)); J = np.zeros(len(self.groups)); kg = np.zeros(len(self.groups))
        for gi, p in enumerate(state):
            if p: s = self.sections[p]; A[gi], I[gi], J[gi], kg[gi] = s['A'], s['I'], s['J'], s['kgPerM']
        present = A[self.el_owner] > 0
        present &= np.where(self.el_owner2 >= 0, A[np.maximum(self.el_owner2, 0)] > 0, True)
        ea = np.where(self.el_link, present * 1.0, A[self.el_owner])
        ei = np.where(self.el_link, present * 1.0, I[self.el_owner])
        ej = np.where(self.el_link, present * 1.0, J[self.el_owner])
        vals = np.repeat(ea, 144) * self.kA + np.repeat(ei, 144) * self.kI + np.repeat(ej, 144) * self.kJ
        K = sp.coo_matrix((vals, (self.rows, self.cols)), shape=(self.ndof, self.ndof)).tocsr() + self.fixed_K
        used = set(self.hoop_nodeset)
        for (i, j), a in zip(self.el_conn, ea):
            if a > 0: used.update((i, j))
        fixed = set(self.fix_dofs) | {6 * k + c for k in range(self.ndof // 6) if k not in used for c in range(6)}
        free = np.array(sorted(set(range(self.ndof)) - fixed))
        try:
            u = np.zeros(self.ndof)
            u[free] = spla.spsolve(K[free][:, free].tocsc(), self.F[free])
        except Exception:
            return None
        if not np.all(np.isfinite(u)) or np.abs(u).max() > 0.5: return None
        rx = [math.degrees(u[6 * k + 3]) for k in self.probe_nodes]
        weight_lb = (float((kg[self.el_owner] * self.el_len).sum()) + self.hoop_mass) / 0.45359237
        tr = bm.TRACKER_TORQUE_LBFT / abs(np.mean(rx))
        return {'rxDeg': rx, 'tr': tr, 'weightLb': weight_lb, 'score': tr / weight_lb}

    def base_state(self):
        """Codex's design with its rule-required core, minus optional tubes that break a
        constraint (e.g. the 5.9 deg front rail joint) or any kink-support coupling fixed."""
        st = []
        for g, ok in zip(self.groups, self.allowed):
            if g['required'] and not g['base']: st.append(ok[0]); continue   # e.g. rear-bay A-frame legs
            if not g['base'] or g.get('blocked'): st.append(None); continue
            bp = next((p for p in self.catalog if p.lower() == g['baseProfile'].lower()), None)
            st.append(bp if bp in ok else ok[0] if ok else None)   # released skeleton tube inside a rule hull: absent
        for gi, g in enumerate(self.groups):
            if st[gi] and not g['required'] and not g.get('skeleton') and any(st[gj] for gj in self.conflicts[gi]):
                st[gi] = None; self.dropped_from_base.append(g['name'])
        for k in self.kinks:  # F.5.2.3: give the first support the member's size if none matches
            if not any(st[s] == st[k['group']] for s in k['supports']):
                s = next((s for s in k['supports'] if self.groups[s]['required'] or st[s]), None)
                if s is not None and st[k['group']] in self.allowed[s]: st[s] = st[k['group']]
        return st

    def _symmetry(self):
        """Mirror (y -> -y) partner of every group; moves always change both. Integer keys
        (0.01 mm) so that y = -0.0 and 0.0 match; -1 means no partner (move forbidden)."""
        def key(g, s):
            q = lambda n: tuple(int(v) for v in np.rint(self.nodes[n] * [1, s, 1] * 1e5))
            return tuple(sorted(tuple(sorted((q(a), q(b)))) for _, a, b in g['members']))
        index = {}
        for gi, g in enumerate(self.groups): index.setdefault(key(g, 1), gi)
        self.mirror = [index.get(key(g, -1), -1) for g in self.groups]
        # Asymmetric tubes (e.g. the supplied bulkhead diagonal) move on their own:
        # symmetry is a search convenience, not a rule.
        self.mirror = [gi if m < 0 else m for gi, m in enumerate(self.mirror)]

    def _joints_and_kinks(self):
        """A15: every suspension node must stay a joint (some tube ends there).
        F.5.2.3: every kink of a multi-piece rule member keeps a same-size support that
        ends at the kink within 30 deg of the member's plane (upper side member exempt;
        brace + lower support and the rear lower path are separate members, G6/G8)."""
        def ends_at(p):
            return {gi for gi, g in enumerate(self.groups) for _, a, b in g['members']
                    if min(np.linalg.norm(self.nodes[a] - p), np.linalg.norm(self.nodes[b] - p)) <= bm.JOINT_CLEARANCE}
        self.pickup_enders = [ends_at(np.array(p)) for p in self.base['loadPoints'] + self.base['fixturePoints']]
        self.kinks = []
        # Codex's rule audit checks the miter of every two-tube group except the upper side
        # member, including the rear lower battery path RF0-BAT0-MH0 and the main brace +
        # lower support (stricter reading of F.5.2.3, rule-constraints.md G6/G8), so the
        # engine enforces the same set. Closed bulkhead frames are not kinked members.
        skip = {'front_bulkhead', 'rear_bulkhead'}
        for gi, g in enumerate(self.groups):
            if not g['base'] or len(g['members']) < 2 or g['role'] == 'side_upper': continue
            if len(g['members']) > 2 and g['role'] in skip: continue
            for (_, a1, b1), (_, a2, b2) in zip(g['members'], g['members'][1:]):
                if b1 != a2: continue
                d1 = self.nodes[b1] - self.nodes[a1]; d2 = self.nodes[b2] - self.nodes[a2]
                nrm = np.cross(d1, d2); s = np.linalg.norm(nrm) / (np.linalg.norm(d1) * np.linalg.norm(d2))
                if s < math.sin(math.radians(1)): continue   # in line: not a kink
                nrm /= np.linalg.norm(nrm); p = self.nodes[b1]; sup = set()
                for gj in ends_at(p) - {gi}:
                    for _, a, b in self.groups[gj]['members']:
                        if min(np.linalg.norm(self.nodes[a] - p), np.linalg.norm(self.nodes[b] - p)) > bm.JOINT_CLEARANCE: continue
                        u = self.nodes[b] - self.nodes[a]; u /= np.linalg.norm(u)
                        if math.degrees(math.asin(min(1, abs(u @ nrm)))) <= KINK_PLANE_DEG: sup.add(gj)
                self.kinks.append({'group': gi, 'node': b1, 'supports': sup})
        self.kinks_of = {}
        for k, kk in enumerate(self.kinks):
            for g in {kk['group']} | kk['supports']: self.kinks_of.setdefault(g, set()).add(k)
        self.pickups_of_group = {}
        for k, en in enumerate(self.pickup_enders):
            for g in en: self.pickups_of_group.setdefault(g, set()).add(k)

    PICKUP = ('FA0', 'FAU', 'FF0', 'FFU', 'RA0', 'RAU', 'RF0', 'RFU')

    def _pickups(self):
        """Each suspension pickup must stay braced in three independent directions."""
        reach = 0.0254 + bm.JOINT_CLEARANCE
        self.pickup_dirs = {}
        for n, p in self.nodes.items():
            if not n.startswith(self.PICKUP) or '@' in n: continue
            dirs = []
            for gi, g in enumerate(self.groups):
                for _, a, b in g['members']:
                    pa, pb_ = self.nodes[a], self.nodes[b]; u = (pb_ - pa) / np.linalg.norm(pb_ - pa)
                    if seg_dist(p, p, pa, pb_) <= reach: dirs.append((gi, u))
            for poly in self.hoop_polys:
                for q0, q1 in zip(poly, poly[1:]):
                    if seg_dist(p, p, q0, q1) <= reach: dirs.append((-1, (q1 - q0) / np.linalg.norm(q1 - q0)))
            self.pickup_dirs[n] = dirs
        self.pickups_of = {}
        for n, dirs in self.pickup_dirs.items():
            for gi, _ in dirs:
                if gi >= 0: self.pickups_of.setdefault(gi, set()).add(n)

    def braced(self, state, n):
        u = np.array([d for gi, d in self.pickup_dirs[n] if gi < 0 or state[gi]])
        return len(u) >= 3 and np.linalg.svd(u, compute_uv=False)[2] > 0.2

    def feasible(self, state, gi, prof):
        """Can the mirror pair of group gi be set to `prof` (None = absent)?"""
        if self.mirror[gi] < 0: return False
        pair = {gi, self.mirror[gi]}
        trial = list(state)
        for g in pair: trial[g] = prof
        return self.valid_change(state, trial, pair)

    def valid_change(self, state, trial, changed):
        for g in changed:
            if trial[g] is None:
                if self.groups[g]['required']: return False
                if any(trial[d] for d in self.dependents.get(g, ())): return False   # tubes resting on it
            else:
                if trial[g] not in self.allowed[g]: return False
                if any(not trial[h] for h in self.depends.get(g, ())): return False  # no mid-air joints
                if any(trial[c] for c in self.conflicts[g]): return False          # crossing / shallow joint
        for k in {k for g in changed for k in self.pickups_of_group.get(g, ())}:
            if not any(trial[g] for g in self.pickup_enders[k]): return False      # pickup stays a joint
        for k in {k for g in changed for k in self.kinks_of.get(g, ())}:
            kk = self.kinks[k]
            if trial[kk['group']] is not None and not any(trial[s] == trial[kk['group']] for s in kk['supports']): return False
        if getattr(self, 'brace', False) and any(trial[g] is None for g in changed):
            if not all(self.braced(trial, n) for g in changed for n in self.pickups_of.get(g, ())): return False
        if self.free:   # F.6.4 / F.6.2.3 on each side the change can touch (rule_structure)
            for side in {s for g in changed for s in self.panel_trigger.get(g, ())}:
                if not self.panel_ok(trial, side, state): return False
        return True

    # -------------------------------------------------------------- free skeleton (F.6.4, F.6.2.3)
    def _panels_setup(self):
        """Per side: the groups that can form a side impact or bulkhead-support panel (any member
        inside rule_structure.region), the groups whose change can matter (those plus every tube
        ending at one of their nodes, the F.5.2.3 supports), and a cache of predicate results."""
        self.ctx = rs.context(bm.main_hoop_path(self.base))
        ends = [(gi, self.nodes[p], self.nodes[q]) for gi, g in enumerate(self.groups)
                for _, a, b in g['members'] for p, q in ((a, b), (b, a))]
        self._end_g = np.array([e[0] for e in ends]); self._end_p = np.array([e[1] for e in ends])
        self._end_o = np.array([e[2] for e in ends])
        self.panel_groups, self.panel_trigger = {}, {}
        for side in (1, -1):
            reg = rs.region(self.ctx, side)
            rel = [gi for gi, g in enumerate(self.groups)
                   if any(rs.in_region(self.nodes[a], self.nodes[b], side, reg) for _, a, b in g['members'])]
            self.panel_groups[side] = rel
            pts = np.array([self.nodes[n] for gi in rel for _, a, b in self.groups[gi]['members'] for n in (a, b)])
            near = np.zeros(len(self._end_p), bool)
            for k in range(0, len(pts), 64):
                near |= np.min(np.linalg.norm(self._end_p[:, None] - pts[None, k:k + 64], axis=2), axis=1) <= bm.JOINT_CLEARANCE
            for gi in set(rel) | set(self._end_g[near].tolist()): self.panel_trigger.setdefault(gi, set()).add(side)
        # Tubes in the bulkhead plane (its top tube sets the F.6.2.3.a top surface), and nodes on a post
        # (hoop or bulkhead side) where a re-routed panel tube may land.
        bx = self.ctx['bulkhead']['x']
        self.bulkhead_groups = {gi for gi, g in enumerate(self.groups)
                                if any(abs(self.nodes[a][0] - bx) < 0.002 and abs(self.nodes[b][0] - bx) < 0.002 for _, a, b in g['members'])}
        names = list(self.nodes); P = np.array([self.nodes[n] for n in names])
        dm, _ = rs._poly_proj(self.ctx['main'], P); df, _ = rs._poly_proj(self.ctx['front'], P)
        bh = self.ctx['bulkhead']
        on_bh = (np.abs(P[:, 0] - bx) <= rs.POST_TOL) & (np.abs(np.abs(P[:, 1]) - bh['y']) <= rs.POST_TOL) & \
                (P[:, 2] >= bh['z0'] - rs.POST_TOL) & (P[:, 2] <= bh['z1'] + rs.POST_TOL)
        self.post_nodes = {n for n, a, b, c in zip(names, dm, df, on_bh) if min(a, b) <= rs.POST_TOL or c}
        self._name_at = {tuple(np.round(p, 4)): n for n, p in self.nodes.items()}
        self._pcache, self._support_cache = {}, {}
        self.panel_evaluations = self.panel_shortcuts = 0

    def _pkey(self, state, side):
        return frozenset((g, rs.level(state[g]), round(rs.section(state[g])['od'], 6))
                         for g in self.panel_groups[side] if state[g] and rs.level(state[g]))

    def _panel_compute(self, state, side):
        """rule_structure.panels on this side's structural panel tubes; structures in group terms."""
        self.panel_evaluations += 1
        ms = [{'a': self.nodes[a], 'b': self.nodes[b], 'profile': state[g], 'tag': g}
              for g in self.panel_groups[side] if state[g] and rs.level(state[g]) for _, a, b in self.groups[g]['members']]
        res = rs.panels(ms, side, self.ctx)
        out = {}
        for key, cls in (('sideImpact', 'B'), ('bulkheadSupport', 'C')):
            out[key] = [{'need': {ms[k]['tag']: (cls, round(rs.section(ms[k]['profile'])['od'], 6))
                                  for k in S['upper'] + S['lower'] + S['diagonals']},
                         'upper': sorted({ms[k]['tag'] for k in S['upper']}),
                         'kinks': [{'node': np.array(k['node']), 'normal': np.array(k['normal']), 'exempt': k['exempt'],
                                    'tags': tuple(ms[i]['tag'] for i in k['members'])} for k in S['kinks']]}
                        for S in res[key]['structures']]
        # Bays left untriangulated (node names), for targeted repairs (layout_engine.skeleton_moves).
        out['deficient'] = [frozenset(self._name_at.get(tuple(np.round(q, 4)), '?') for q in face)
                            for key in self.PANELS for face in res[key]['deficient']]
        # A structure found here still stands after any change that keeps its own tubes (class and OD),
        # the bulkhead's tubes and the side impact upper members (F.6.2.3.a/b heights) as they are.
        out['deps'] = {g: self._lvl_od(state, g) for g in (self.bulkhead_groups & set(self.panel_groups[side]))
                       | {g for S in out['sideImpact'] for g in S['upper']}}
        return out

    @staticmethod
    def _lvl_od(state, g):
        return (rs.level(state[g]), round(rs.section(state[g])['od'], 6)) if state[g] else (None, None)

    def _holds(self, S, state):
        return all(state[g] and rs.RANK[rs.level(state[g])] >= rs.RANK[c] and round(rs.section(state[g])['od'], 6) == od
                   for g, (c, od) in S['need'].items())

    def _supporters(self, node):
        key = tuple(np.round(node, 6))
        r = self._support_cache.get(key)
        if r is None:
            sel = np.flatnonzero(np.linalg.norm(self._end_p - node, axis=1) <= bm.JOINT_CLEARANCE)
            r = self._support_cache[key] = [(int(self._end_g[i]), rs._unit(self._end_o[i] - self._end_p[i])) for i in sel]
        return r

    def _kink_ok(self, k, state):
        """rule_structure.kink_ok in group terms (F.5.2.3 support of one kink)."""
        p1, p2 = (state[t] for t in k['tags'])
        if not p1 or not p2: return False
        s1, s2 = rs.section(p1), rs.section(p2)
        same = abs(s1['od'] - s2['od']) < 1e-9 and abs(s1['wall'] - s2['wall']) < 1e-9
        for g, u in self._supporters(k['node']):
            if g in k['tags'] or not state[g]: continue
            if k['exempt']:
                if rs.level(state[g]): return True
                continue
            s = rs.section(state[g])
            if same and abs(s['od'] - s1['od']) < 1e-9 and abs(s['wall'] - s1['wall']) < 1e-9 \
                    and math.degrees(math.asin(min(1.0, abs(u @ k['normal'])))) <= rs.PLANE_DEG:
                return True
        return False

    PANELS = ('sideImpact', 'bulkheadSupport')

    def panel_ok(self, state, side, ref=None):
        """Side impact and bulkhead-support structure both present on this side, kinks supported.
        ref: a state already checked (the one a move starts from); its structures are reused when
        the change leaves them standing (rule_structure guarantees extra tubes never undo one)."""
        key = self._pkey(state, side)
        res = self._pcache.get(key)
        if res is None and ref is not None:
            r0 = self._pcache.get(self._pkey(ref, side))
            if r0 is not None and all(self._lvl_od(state, g) == v for g, v in r0['deps'].items()):
                if all(any(self._holds(S, state) and all(self._kink_ok(k, state) for k in S['kinks']) for S in r0[p])
                       for p in self.PANELS):
                    self.panel_shortcuts += 1
                    return True
        if res is None: res = self._pcache[key] = self._panel_compute(state, side)
        return all(any(all(self._kink_ok(k, state) for k in S['kinks']) for S in res[p]) for p in self.PANELS)

    def panels_ok(self, state):
        return not self.free or all(self.panel_ok(state, s) for s in (1, -1))

    def panel_defects(self, state):
        """What blocks a legal structure on each failing side: the untriangulated bays (node-name
        sets) and the kinks of otherwise legal structures that lack an F.5.2.3 support."""
        faces, kinks = [], []
        for side in (1, -1):
            if self.panel_ok(state, side): continue
            res = self._pcache[self._pkey(state, side)]
            faces += res['deficient']
            kinks += [k for p in self.PANELS for S in res[p] for k in S['kinks'] if not self._kink_ok(k, state)]
        return faces, kinks

    def witness(self, state):
        """Groups of the first passing side impact and bulkhead-support structure on each side."""
        out = set()
        for side in (1, -1):
            res = self._pcache.get(self._pkey(state, side))
            if res is None: self.panel_ok(state, side); res = self._pcache[self._pkey(state, side)]
            for p in self.PANELS:
                S = next((S for S in res[p] if all(self._kink_ok(k, state) for k in S['kinks'])), None)
                if S: out |= set(S['need'])
        return out

    def close(self, state, changes):
        """Compound move: `changes` plus removing every present optional tube that conflicts with a
        tube it adds, and every tube resting on a removed one (mirror pairs throughout). None if a
        required tube would have to go."""
        ch = dict(changes); trial = list(state)
        for g, p in ch.items(): trial[g] = p
        todo = [c for g, p in changes.items() if p is not None for c in self.conflicts[g] if trial[c] is not None and c not in changes]
        while todo:
            c = todo.pop()
            for x in {c, self.mirror[c]}:
                if trial[x] is None: continue
                if self.groups[x]['required'] or x in changes: return None
                trial[x] = None; ch[x] = None
                todo.extend(d for d in self.dependents.get(x, ()) if trial[d] is not None)
        return ch

    def valid_state(self, state):
        return self.valid_change(state, state, set(range(len(self.groups))))


# ------------------------------------------------------------------ search
def greedy(pb, state, log, budget_s):
    best = pb.evaluate(state); t0 = time.time(); evals = 0
    while time.time() - t0 < budget_s:
        move = None
        for gi in range(len(pb.groups)):
            if pb.mirror[gi] < gi: continue
            options = [None] + pb.allowed[gi]
            for prof in options:
                if prof == state[gi] or not pb.feasible(state, gi, prof): continue
                trial = list(state); trial[gi] = trial[pb.mirror[gi]] = prof
                r = pb.evaluate(trial); evals += 1
                if r and r['score'] > best['score'] * (1 + 1e-6) and (move is None or r['score'] > move[2]['score']):
                    move = (gi, prof, r)
        if not move: break
        gi, prof, best = move; state[gi] = state[pb.mirror[gi]] = prof
        log.append({'group': pb.groups[gi]['name'], 'profile': prof, **{k: best[k] for k in ('tr', 'weightLb', 'score')}})
        print(f"  {pb.groups[gi]['name']:<34} -> {prof or 'removed':<22} TR {best['tr']:7.1f}  {best['weightLb']:5.1f} lb  ratio {best['score']:.3f}", flush=True)
    return state, best, evals


def brace_pickups(pb, state, log):
    """Add the best-scoring tube pair that braces each under-braced suspension pickup."""
    for n in sorted(pb.pickup_dirs):
        if pb.braced(state, n): continue
        best = None
        for gi in sorted({g for g, _ in pb.pickup_dirs[n] if g >= 0}):
            if state[gi] is not None: continue
            for prof in pb.allowed[gi]:
                if not pb.feasible(state, gi, prof): continue
                trial = list(state); trial[gi] = trial[pb.mirror[gi]] = prof
                if not pb.braced(trial, n): continue
                r = pb.evaluate(trial)
                if r and (best is None or r['score'] > best[2]['score']): best = (gi, prof, r)
        if best is None: print(f'  WARNING: no feasible tube braces pickup {n}'); continue
        gi, prof, r = best; state[gi] = state[pb.mirror[gi]] = prof
        log.append({'group': pb.groups[gi]['name'], 'profile': prof, 'reason': 'brace pickup ' + n, **{k: r[k] for k in ('tr', 'weightLb', 'score')}})
        print(f"  brace {n:<6} with {pb.groups[gi]['name']:<34} {prof:<22} ratio {r['score']:.3f}", flush=True)
    return state


def search(pb, restarts, seed, budget_s):
    rng = random.Random(seed)
    state = pb.base_state(); start = pb.evaluate(state)
    print(f"Base design: TR {start['tr']:.1f} lb-ft/deg, {start['weightLb']:.1f} lb, ratio {start['score']:.3f}")
    log = []
    state = brace_pickups(pb, state, log)
    state, best, evals = greedy(pb, state, log, budget_s)
    best_state = list(state)
    for r in range(restarts):
        trial = list(best_state)
        opt = [gi for gi, g in enumerate(pb.groups) if not g['required']]
        for gi in rng.sample(opt, k=min(6, len(opt))):  # perturb: toggle a few optional tube pairs
            prof = None if trial[gi] else rng.choice(pb.allowed[gi])
            if pb.feasible(trial, gi, prof): trial[gi] = trial[pb.mirror[gi]] = prof
        if pb.evaluate(trial) is None: continue
        print(f'-- restart {r + 1}')
        trial, res, n = greedy(pb, trial, [], budget_s); evals += n
        if res['score'] > best['score']:
            best, best_state = res, trial; print(f'   new best ratio {best["score"]:.3f}')
    return start, best_state, best, log, evals


# ------------------------------------------------------------------ recipe out
def cad_name(name, xbraces):
    """SolidWorks-safe member name: no '@' (dimension syntax) or other reserved characters;
    X-brace members become XBrace<k>_T (through tube) / _A / _B (halves)."""
    if name[:3] in ('XT_', 'XA_', 'XB_'):
        k = xbraces.setdefault(name[3:], len(xbraces) + 1)
        return f'XBrace{k}_' + {'XT_': 'T', 'XA_': 'A', 'XB_': 'B'}[name[:3]]
    name = name.replace('Cand_', 'Add_', 1) if name.startswith('Cand_') else name
    return re.sub(r'[^A-Za-z0-9_-]', '-', name.replace('@', '-'))


def to_recipe(pb, state):
    groups, members, xbraces = [], [], {}
    for g, prof in zip(pb.groups, state):
        if prof is None: continue
        ms = []
        for long_name, a, b in g['members']:
            name = cad_name(long_name, xbraces)
            role = g['role']
            # Declared class: the rule class for required tubes; for optional tubes the best
            # class the chosen profile meets, or NS (Non-Structural, counts toward nothing).
            size = g['cls'] or next((c for c in ('A', 'B', 'C') if meets(prof, c)), 'NS')
            m = {'name': name, 'a': pb.nodes[a].tolist(), 'b': pb.nodes[b].tolist(), 'nodes': [a, b], 'role': role,
                 'size': size, 'profile': prof, 'order': len(members)}
            if name != long_name: m['optimizerName'] = long_name
            ms.append(m); members.append(m)
        if g['role'] == 'x_brace':   # through tube + two halves: separate weldment groups (not end-to-end)
            groups.extend({'name': 'Weld_' + m['name'], 'profile': prof, 'members': [m], 'miter': False} for m in ms)
        else:
            gname = g['name'] if g['base'] else 'Weld_' + ms[0]['name'] if len(ms) == 1 else re.sub(r'[^A-Za-z0-9_-]', '-', g['name'])
            groups.append({'name': gname, 'profile': prof, 'members': ms, 'miter': len(ms) > 1})
    r = dict(pb.base)
    if getattr(pb, 'halfshaft', None):
        axle = getattr(pb, 'axle', None)
        r['halfshaftCorridor'] = {**pb.halfshaft, 'yInM': 0.150, 'yOutM': 0.260,
                                  'basis': axle['basis'] if axle else 'estimated: midway between rear suspension pickups'}
        if axle: r['halfshaftCorridor'].update(frac=axle['frac'], rearFrameApexT=pb.apex_t)
    if getattr(pb, 'accumulator', None): r['accumulatorBox'] = pb.accumulator
    r['photoKeepouts'] = {'file': photo_keepouts.OUT.relative_to(ROOT).as_posix(),
                          'baseConflicts': pb.photo_conflicts}
    r.update({'status': 'ground-structure result; native rule/driver/trim/study gates required',
              'nodes': {k: v.tolist() for k, v in pb.nodes.items()},
              'groups': groups, 'members': members,
              'trims': make_trims(groups, {k: v.tolist() for k, v in pb.nodes.items()}, pb.hoop_nodes)})
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base', default='design/frame-005.json')
    ap.add_argument('--out')
    ap.add_argument('--catalog', choices=list(CATALOG), default='round1')
    ap.add_argument('--restarts', type=int, default=8)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--budget', type=float, default=600, help='seconds per greedy run')
    a = ap.parse_args()
    base = json.loads((ROOT / a.base).read_text(encoding='utf-8-sig'))
    t = time.time(); pb = Problem(base, CATALOG[a.catalog])
    nreq = sum(g['required'] for g in pb.groups); ncand = sum(not g['base'] for g in pb.groups)
    print(f"{len(pb.groups)} groups: {nreq} required, {len(pb.groups) - nreq - ncand} optional base, {ncand} new candidates "
          f"({len(pb.rejected)} pairs rejected by keep-outs/templates/crossings); model {pb.ndof} DOF, built in {time.time() - t:.1f}s")
    start, state, best, log, evals = search(pb, a.restarts, a.seed, a.budget)
    print(f"Best: TR {best['tr']:.1f} lb-ft/deg, {best['weightLb']:.1f} lb, ratio {best['score']:.3f} "
          f"(base {start['score']:.3f}); {evals} evaluations in {time.time() - t:.0f}s")
    unbraced = [n for n in pb.pickup_dirs if not pb.braced(state, n)]
    print('Unbraced suspension pickups:', unbraced or 'none')
    if a.out:
        rec = to_recipe(pb, state)
        rec['groundStructure'] = {'base': a.base, 'catalog': a.catalog, 'seed': a.seed, 'restarts': a.restarts,
                                  'fastModel': {'base': start, 'best': best}, 'moves': log,
                                  'candidateCount': ncand, 'rejectedPairs': len(pb.rejected), 'unbracedPickups': unbraced,
                                  'constraints': ['rule size classes', 'keep-outs and driver/cockpit template sweeps', 'no crossing tubes',
                                                  f'joint angle >= {MIN_ANGLE_DEG} deg', 'left/right symmetric', 'suspension pickups braced in 3 directions']}
        (ROOT / a.out).write_text(json.dumps(rec, indent=2) + '\n', encoding='utf-8')
        print(f"Wrote {a.out}: {len(rec['groups'])} groups, {len(rec['members'])} members, {len(rec['trims'])} trims")


if __name__ == '__main__':
    main()
