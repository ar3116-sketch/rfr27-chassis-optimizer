"""Packaging keep-outs estimated from photos of the team's previous car (#61), 19 Oct 2026.

Each zone is placed from datums already in this model (accumulator box, upright ball joints,
main hoop, driver template, front-brace line), never from photo pixels alone, and is padded.
The basis and confidence of each one are recorded with it. These are packaging assumptions,
not measured components: replace them as real dimensions arrive.

Writes design/keepouts-photo-001.json; ground_structure.Forbidden loads that file.
    python scripts/photo_keepouts.py            # regenerate the zone file
    python scripts/photo_keepouts.py --check design/frame-009c.json design/frame-009b.json
"""
from pathlib import Path
import argparse, json, sys
import numpy as np
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from frame_geometry import source_geometry
import beam_model as bm

OUT = ROOT / 'design/keepouts-photo-001.json'
LAYOUT = ROOT / 'design/keepouts-photo-001-layout.json'   # native packaging-audit envelopes
PHOTOS = 'user photos of car #61, 19 Oct 2026 (rear, cockpit, nose-open, side, underside x2)'


def box(x0, x1, y0, y1, z0, z1):
    return [[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]


def cyl_y(x, z, r, y0, y1, n=24):
    """Cylinder with its axis along Y."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.stack([x + r * np.cos(a), np.zeros(n), z + r * np.sin(a)], 1)
    return np.vstack([ring + [0, y0, 0], ring + [0, y1, 0]]).tolist()


def mirror(pts):
    return [[x, -y, z] for x, y, z in pts]


def zones():
    sk, bodies = source_geometry()
    body = {b['name']: np.array(b['verticesM']) for b in bodies}
    bat = body['Boss-Extrude1']; blo, bhi = bat.min(0), bat.max(0)
    # Upright ball joints (outer A-arm apexes) locate the wheel centres.
    def apex(name):
        pts = [tuple(np.round(q[k], 6)) for q in sk[name] for k in ('start', 'end')]
        return [np.array(p) for p in set(pts) if abs(p[1]) > 0.45]
    fl, rl = apex('FL A-Arm'), apex('RL A-Arm')
    fx = float(np.mean([p[0] for p in fl])); fz = float(np.mean([p[2] for p in fl]))
    rx = float(np.mean([p[0] for p in rl if p[0] > 0])); rz = float(np.mean([p[2] for p in rl if p[0] > 0]))
    fy_out = float(min(p[1] for p in fl)) - 0.02
    driver = json.loads((ROOT / 'design/driver-checks-009c.json').read_text(encoding='utf-8-sig'))
    shoulder = next(c for c in driver['driverCircles'] if c['name'] == 'RuleShoulder')['centerM']
    # Main hoop rear face at shoulder height, right side (driver's right = -Y).
    hoop = bm.hoops('source')   # the team's Main Roll Hoop Sketch (required from 009d on)
    mh = np.array(next(v for k, v in hoop.items() if 'Main' in k))
    side = mh[mh[:, 1] < -0.1]; side = side[np.argsort(side[:, 2])]
    hoop_x, hoop_y = (float(np.interp(shoulder[2], side[:, 2], side[:, i])) for i in (0, 1))
    # Front hoop-brace line (top of the nose structure), from the front hoop top to the bulkhead top.
    fb = json.loads((ROOT / 'design/frame-009c.json').read_text(encoding='utf-8-sig'))['nodes']
    (x0, z0), (x1, z1) = (fb['FHBL'][0], fb['FHBL'][2]), (fb['FBUL'][0], fb['FBUL'][2])
    brace_z = lambda x: z0 + (z1 - z0) * (x - x0) / (x1 - x0)

    Z = []
    def add(name, pts, basis, confidence, margin=0.0, sym=False, **extra):
        e = {'name': name, 'verticesM': [[round(c, 6) for c in p] for p in pts], 'marginM': margin,
             'basis': basis, 'confidence': confidence, 'enabled': True, **extra}
        if sym:
            Z.append({**e, 'name': name + 'L'})
            Z.append({**e, 'name': name + 'R', 'verticesM': [[round(c, 6) for c in p] for p in mirror(pts)]})
        else:
            Z.append(e)

    add('AccumulatorDropPath', box(blo[0], bhi[0], blo[1], bhi[1], -0.05, blo[2] + 0.001),
        'User, 19 Oct: the accumulator is removed by dropping it out the bottom. Its full footprint '
        '(Boss-Extrude1, x/y unchanged) must stay open from its floor down past the lowest tube to the ground. '
        'Footprint not enlarged: the fixed main-hoop base tube is about 9 mm ahead of it.',
        'high (user statement; footprint from the supplied box)', margin=0.0,
        rule='F.10 / EV.5 container must be removable; packaging requirement')

    # Tie rod: one concrete steering assumption (rev 2, 20 Oct). The first version was a
    # +/-60 x +/-45 mm "somewhere near the axle" box; no straight F.6.2.3 diagonal can cross the
    # front side bay without entering it. A tie rod has to pass through one triangle of that bay,
    # so it is placed rear-steer, 47 mm behind the axle at z 192 mm, which is the point of largest
    # clearance inside the FA0-FF0-FAU triangle (grid search, 2 mm steps, 16 mm to the nearest
    # tube). Swept envelope: 20 mm radius at the rack end (y 150 mm; rod plus small motion
    # near the inner joint), 40 mm at the steering arm (bump +/-25 mm and steer travel).
    xt, zt = fx - 0.047, 0.192
    a = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    ring = lambda r, y: np.stack([xt + r * np.cos(a), np.full(24, y), zt + r * np.sin(a)], 1)
    add('TieRodCorridor', np.vstack([ring(0.020, 0.150), ring(0.040, fy_out)]).tolist(),
        f'Photos 2/3/6: rack across the footwell, tie rods leave through the frame side between the A-arm rails. '
        f'Assumed rear steer: tie-rod line x {xt*1e3:.0f} mm (47 mm behind the axle at {fx*1e3:.0f}), z {zt*1e3:.0f} mm '
        f'(ball-joint midpoint {fz*1e3:.0f}), from the rack end at y 150 mm to the steering arm at y {fy_out*1e3:.0f} mm. '
        'Swept radius 20 mm inboard to 40 mm at the upright. Position chosen to pass through the triangle that '
        'the rule-required FrontDiagonalMiddle (F.6.2.3d) leaves open.',
        'medium (steering geometry assumed: rear steer, arm 47 mm aft; replace when the steering layout is known)', sym=True,
        revision=2, supersedes='rev 1 box x 1509-1629, z 160-242 (no F.6.2.3 diagonal can clear it)')

    xs = (1.500, 1.660)
    pts = [[x, y, brace_z(x) + dz] for x in xs for y in (-0.260, 0.260) for dz in (0.025, 0.150)]
    add('FrontShockRockerPackage', pts,
        'Photos 2/3/6: two front dampers lie across the car on top of the nose just ahead of the front hoop, '
        'rockers at their outer ends. Modelled above the front hoop-brace line (+25 to +150 mm), '
        'x 1500-1660 mm, y +/-260 mm. In-plane bracing between the braces stays allowed.',
        'medium-low (layout copied from car 61; new pushrod geometry not supplied)')

    add('RearShockRockerPackage', box(rx - 0.105, rx + 0.045, -0.240, 0.240, 0.415, 0.530),
        'Photo 1: two rear dampers lie across the top of the rear frame, rockers at the outer ends, just aft of '
        f'the main-hoop brace feet. Centred on the rear upright x {rx*1e3:.0f} mm, kept aft of the brace '
        'feet (x 100), above the rear upper rails (z 415-530).',
        'medium-low (layout copied from car 61)')

    sx0 = hoop_x - 0.125; sx1 = hoop_x - 0.040  # hoop leans back: keep its rear face clear over the full height
    add('MasterSwitchPanel', box(sx0, sx1, hoop_y - 0.060, hoop_y + 0.050, shoulder[2] - 0.060, shoulder[2] + 0.060),
        f'T.9.3.1: GLVMS and TSMS on the driver\'s right, near the main hoop, at shoulder height, operable from '
        f'outside (photo 4 shows them on the right side panel just aft of the hoop). Shoulder z {shoulder[2]*1e3:.0f} '
        f'mm (rule template), hoop at x {hoop_x*1e3:.0f}, y {hoop_y*1e3:.0f}; 120 mm tall switch bodies, '
        '110 mm deep across the side (60 mm outboard of the hoop line to 50 mm inboard).', 'high for location (rule), medium for size', rule='T.9.3.1, EV.8.9.1')

    r = 0.130
    add('ChainSprocketShield', cyl_y(rx, rz, r, -0.155, -0.095) + cyl_y(rx + 0.180, rz + 0.010, 0.060, -0.155, -0.095),
        f'Photo 7: chain drive to a large sprocket on the differential. T.5.2.7: steel scatter shield, '
        f'3x chain width, around the whole chain run. Sprocket+shield radius {r*1e3:.0f} mm on the rear axle '
        f'(x {rx*1e3:.0f}, z {rz*1e3:.0f}), chain run forward to an assumed motor sprocket 180 mm ahead; '
        'right side (-Y) as on car 61. Only the part below Loft8 adds anything.',
        'low (keep only if the chain drive is retained; side and sprocket size assumed)', rule='T.5.2')

    return Z, {'frontWheelCentreM': [fx, fz], 'rearWheelCentreM': [rx, rz],
               'shoulderM': shoulder, 'mainHoopRightAtShoulderM': [hoop_x, hoop_y]}


def write():
    Z, datums = zones()
    doc = {'schema': 'rfr.keepouts.photo/1', 'source': PHOTOS, 'units': 'm, X forward, Y left, Z up',
           'status': 'estimated packaging keep-outs; not measured components; not in the native SolidWorks audit yet',
           'datums': datums, 'zones': Z}
    OUT.write_text(json.dumps(doc, indent=2) + '\n', encoding='utf-8')
    # The same zones as native packaging-audit envelopes: every zone spans two Y planes, so it is
    # its XZ convex hull extruded along +Y (PackagingAudit.PolygonPrism).
    env = []
    for e in Z:
        v = np.array(e['verticesM']); y0, y1 = v[:, 1].min(), v[:, 1].max()
        assert np.allclose(np.sort(np.unique(np.round(v[:, 1], 9))), [round(y0, 9), round(y1, 9)]), e['name']
        p0, p1 = v[np.isclose(v[:, 1], y0)][:, [0, 2]], v[np.isclose(v[:, 1], y1)][:, [0, 2]]
        same = len(p0) == len(p1) and {tuple(q) for q in np.round(p0, 9)} == {tuple(q) for q in np.round(p1, 9)}
        # A tapered zone (tie-rod frustum, rings in matching order) becomes Y slabs, each the XZ hull
        # of the sections at both slab ends: never smaller than the true zone.
        n = 1 if same else 8
        for k in range(n):
            t0, t1 = k / n, (k + 1) / n
            xz = np.vstack([p0 + (p1 - p0) * t0, p0 + (p1 - p0) * t1]) if not same else p0
            h = ConvexHull(xz); ya = y0 + (y1 - y0) * t0
            env.append({'name': 'Photo_' + e['name'] + ('' if n == 1 else f'_s{k}'),
                        'polygon': [[float(xz[i, 0]), float(ya), float(xz[i, 1])] for i in h.vertices],
                        'direction': [0, 1, 0], 'depthM': float((y1 - y0) / n), 'test': 'photo-estimated packaging keep-out',
                        'conservativeShape': 'exact prism of the zone hull (Y extrusion)' if n == 1 else
                                             'Y slab of a tapered zone, hull of both end sections (covers the zone)'})
    LAYOUT.write_text(json.dumps({'schema': 'rfr.keepouts.photo.layout/1', 'source': str(OUT.relative_to(ROOT).as_posix()),
                                  'sweptEnvelopes': env, 'driverCircles': []}, indent=2) + '\n', encoding='utf-8')
    return doc


def load(enabled_only=True):
    """Enabled zones. PHOTO_KEEPOUTS_OFF=Name1,Name2 (environment) drops zones by name prefix,
    for sensitivity runs only; the zone file is never changed."""
    if not OUT.exists(): return []
    import os
    off = [n for n in os.environ.get('PHOTO_KEEPOUTS_OFF', '').split(',') if n]
    z = json.loads(OUT.read_text(encoding='utf-8'))['zones']
    return [e for e in z if (e.get('enabled', True) or not enabled_only) and not any(e['name'].startswith(n) for n in off)]


def seg_hull_min(a, b, eq, t0=0.03, t1=0.97):
    """Exact minimum over t in [t0, t1] of max_i(n_i . (a + t (b - a)) + d_i) for a convex hull's
    facet equations eq: the signed distance of the segment's centreline to the hull (at most the
    Euclidean distance, so a clearance from it is never optimistic). The function is convex and
    piecewise linear in t, so the minimum lies at an end or where two facets' values cross. Sampling
    every 1 mm (until 20 Oct) missed grazing contacts: a 0.750 in tube reported 0.2 mm clear of the
    switch zone was 0.06 mm inside it."""
    base = eq[:, :3] @ a + eq[:, 3]; slope = eq[:, :3] @ (b - a)
    if len(eq) <= 64:
        i, j = np.triu_indices(len(eq), 1)
        ds = slope[i] - slope[j]; ok = np.abs(ds) > 1e-15
        t = (base[j][ok] - base[i][ok]) / ds[ok]
        T = np.concatenate([[t0, t1], t[(t > t0) & (t < t1)]])
        return float(np.min(np.max(base[None, :] + T[:, None] * slope[None, :], axis=1)))
    # many facets (the driver outline has 516): bracket on a coarse grid, then golden-section search,
    # exact for a convex function to ~1e-13 of the segment length
    T = np.linspace(t0, t1, 65); v = np.max(base[None, :] + T[:, None] * slope[None, :], axis=1)
    k = int(np.argmin(v)); lo, hi = T[max(k - 1, 0)], T[min(k + 1, 64)]
    f = lambda t: float(np.max(base + t * slope)); g = 0.3819660112501051
    x1, x2 = lo + g * (hi - lo), hi - g * (hi - lo); f1, f2 = f(x1), f(x2)
    for _ in range(64):
        if f1 <= f2: hi, x2, f2 = x2, x1, f1; x1 = lo + g * (hi - lo); f1 = f(x1)
        else: lo, x1, f1 = x1, x2, f2; x2 = hi - g * (hi - lo); f2 = f(x2)
    return min(float(v[k]), f1, f2)


def check(frame_path, zone_list=None):
    return check_recipe(json.loads(Path(frame_path).read_text(encoding='utf-8-sig')), zone_list)


def check_recipe(d, zone_list=None, step=None):
    """Signed surface clearance (mm) of every member and roll-hoop segment to every zone;
    member ends excluded as in ground_structure. Returns (hits, per-zone minimum). Exact
    (seg_hull_min) unless `step` (m) asks for the older sampled check."""
    zone_list = zone_list or load()
    hulls = [(e['name'], ConvexHull(np.array(e['verticesM'])).equations, e.get('marginM', 0.0)) for e in zone_list]
    hits, worst = [], {}
    hoop_members = [{'name': f'{h}[{i}]', 'a': p[i], 'b': p[i + 1], 'profile': bm.HOOP_PROFILE}
                    for h, p in bm.hoops(bm.main_hoop_path(d)).items() for i in range(len(p) - 1)]
    for m in d['members'] + hoop_members:
        a, b = np.array(m['a']), np.array(m['b'])
        r = bm.section(m['profile'])['od'] / 2
        if step:
            n = max(2, int(np.linalg.norm(b - a) / step))
            pts = a + np.linspace(0.03, 0.97, n)[:, None] * (b - a)
        for name, eq, margin in hulls:
            raw = float(np.min(np.max(pts @ eq[:, :3].T + eq[:, 3], axis=1))) if step else seg_hull_min(a, b, eq)
            c = raw - r - margin
            if name not in worst or c < worst[name][0]: worst[name] = (c, m['name'])
            if c < 0: hits.append({'zone': name, 'member': m['name'], 'clearanceMm': round(c * 1e3, 2)})
    return hits, {k: {'minClearanceMm': round(v[0] * 1e3, 2), 'member': v[1]} for k, v in worst.items()}


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--check', nargs='*')
    a = ap.parse_args()
    doc = write(); print(f"wrote {OUT.relative_to(ROOT)}: {len(doc['zones'])} zones; datums {doc['datums']}")
    for f in a.check or []:
        hits, worst = check(ROOT / f)
        print(f'\n{f}: {len(hits)} member/zone intersections')
        for h in sorted(hits, key=lambda h: h['clearanceMm']): print('  ', h)
        for k, v in worst.items(): print(f'   {k:28s} min {v["minClearanceMm"]:7.1f} mm  ({v["member"]})')
