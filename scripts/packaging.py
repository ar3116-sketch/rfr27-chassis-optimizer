"""Packaging inputs of the optimizer: the accumulator keep-out box and the rear axle line.

Defaults reproduce what the optimizer used through candidate 009c: the starter keep-out solid
Boss-Extrude1 (18.0 x 19.3 x 9.8 in) and a halfshaft corridor on the mean of the four rear
suspension pickups. Team answers of 12 Oct 2026: the accumulator is about 19-21 in wide and
about 19 in long, in the bay behind the main hoop; the rear axle sits toward the middle/front
of the rear suspension box. Both are scenario inputs until the team gives exact numbers.

  accumulator_box(width_in=20)            -> box dict (lo/hi corners in metres)
  axle_line(nodes, frac=0.7)               -> {'xM', 'zM', ...}
  battery_checks(recipe, box)              -> F.11.1.1 / F.11.2 checks of a recipe
"""
import math
import numpy as np
from scipy.spatial import Delaunay

import beam_model as bm
from frame_geometry import source_geometry, outer_radius

IN = 0.0254
ACCUMULATOR = 'Boss-Extrude1'    # starter keep-out solid of the tractive battery
POWERTRAIN = 'Loft8'             # starter keep-out solid of the motor/gearbox (rearmost)
HOOP_R = 0.0127                  # 1.000 in Size A hoop tube


def _solid(name):
    _, refs = source_geometry()
    v = np.array(next(b['verticesM'] for b in refs if b['name'] == name))
    return v.min(axis=0), v.max(axis=0)


def hoop_leg_limits(z_lo, z_hi, main_hoop='source'):
    """Inner half-width of the main-hoop legs (tube surface) over z_lo..z_hi, and the
    smallest x of the legs' aft surface there (the box must stay behind it if it is wider
    than the legs)."""
    poly = np.array(bm.hoops(main_hoop)['MainHoop_1'])
    pts = []
    for a, b in zip(poly, poly[1:]):
        for t in np.linspace(0, 1, 50):
            p = a + t * (b - a)
            if z_lo - HOOP_R <= p[2] <= z_hi + HOOP_R: pts.append(p)
    pts = np.array(pts)
    # The legs lean aft by about 10 deg; the aft surface is HOOP_R / cos(lean) behind the centreline.
    return float(np.min(np.abs(pts[:, 1]))) - HOOP_R, float(np.min(pts[:, 0])) - HOOP_R / math.cos(math.radians(10))


def accumulator_box(width_in=None, length_in=None, front_x=None, main_hoop='source', hoop_gap=0.001):
    """Accumulator keep-out box. None for every argument returns the starter solid exactly.
    Width is across the car (y, centred), length along it (x); height and z stay the starter's.
    Placement when front_x is None: the starter front face (x 761.9 mm, between the main-hoop
    legs) if the box fits between the legs with hoop_gap each side; otherwise the box moves aft
    so its rear face meets the powertrain keep-out (Loft8), the most room the lower main-brace
    supports give a box wider than the hoop legs (they diverge from the hoop base)."""
    lo, hi = _solid(ACCUMULATOR)
    if width_in is None and length_in is None and front_x is None:
        return {'name': ACCUMULATOR, 'lo': lo.tolist(), 'hi': hi.tolist(), 'widthIn': (hi[1] - lo[1]) / IN,
                'lengthIn': (hi[0] - lo[0]) / IN, 'heightIn': (hi[2] - lo[2]) / IN, 'placement': 'starter solid',
                'basis': 'starter keep-out solid Boss-Extrude1'}
    W = (width_in * IN) if width_in is not None else hi[1] - lo[1]
    L = (length_in * IN) if length_in is not None else hi[0] - lo[0]
    half_legs, legs_aft_x = hoop_leg_limits(lo[2], hi[2], main_hoop)
    if front_x is not None: placement = 'given front face'
    elif W / 2 <= half_legs - hoop_gap: front_x, placement = hi[0], 'starter front face, between the main-hoop legs'
    else:
        front_x = _solid(POWERTRAIN)[1][0] + L
        placement = 'rear face on the powertrain keep-out (wider than the main-hoop legs)'
        if front_x > legs_aft_x - hoop_gap: front_x = legs_aft_x - hoop_gap
    box = {'name': ACCUMULATOR, 'lo': [front_x - L, -W / 2, float(lo[2])], 'hi': [front_x, W / 2, float(hi[2])],
           'widthIn': W / IN, 'lengthIn': L / IN, 'heightIn': (hi[2] - lo[2]) / IN, 'placement': placement,
           'basis': f'scenario: {W / IN:.1f} x {L / IN:.1f} in, z and height of the starter solid'}
    pt_x = _solid(POWERTRAIN)[1][0]
    if box['lo'][0] < pt_x - 1e-6:
        raise ValueError(f'Accumulator box ({W / IN:.1f} x {L / IN:.1f} in) overlaps the powertrain keep-out by '
                         f'{(pt_x - box["lo"][0]) * 1000:.1f} mm')
    return box


def box_vertices(box):
    lo, hi = np.array(box['lo']), np.array(box['hi'])
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def axle_line(nodes, frac=None, zM=None):
    """Rear axle (halfshaft corridor axis, along y). frac None: the mean of the four rear pickups
    (the estimate used through 009c). frac f: f of the way from the aft pickups (RA0/RAU) to
    the fore pickups (RF0/RFU). zM None: the mean height of the four pickups."""
    P = {k: np.array(nodes[k]) for k in ('RA0L', 'RAUL', 'RF0L', 'RFUL')}
    z = zM
    z0 = float(np.mean([p[2] for p in P.values()])) if z is None else float(z)
    if frac is None:
        return {'xM': float(np.mean([p[0] for p in P.values()])), 'zM': z0, 'frac': None,
                'basis': 'estimated: midway between rear suspension pickups'}
    xa = (P['RA0L'][0] + P['RAUL'][0]) / 2; xf = (P['RF0L'][0] + P['RFUL'][0]) / 2
    return {'xM': float(xa + frac * (xf - xa)), 'zM': z0, 'frac': float(frac),
            'basis': f'scenario: {frac * 100:.0f}% of the way from the aft to the fore rear pickups'
                     + ('' if z is None else f', z {z0 * 1000:.1f} mm')}


def _seg_box_dist(a, b, lo, hi, step=0.001):
    n = max(2, int(np.linalg.norm(b - a) / step)); P = a + np.linspace(0, 1, n)[:, None] * (b - a)
    return float(np.linalg.norm(np.maximum(np.maximum(lo - P, 0), P - hi), axis=1).min())


def _inside_polygon(pts, poly):
    """Even-odd test of 2D points against a closed polygon."""
    x, y = pts[:, 0], pts[:, 1]; inside = np.zeros(len(pts), bool)
    for (x1, y1), (x2, y2) in zip(poly, np.roll(poly, -1, axis=0)):
        cross = (y1 > y) != (y2 > y)
        xi = x1 + (y - y1) * (x2 - x1) / np.where(y2 != y1, y2 - y1, 1)
        inside ^= cross & (x < xi)
    return inside


def battery_checks(rec, box):
    """Accumulator rules checked on a recipe (centrelines; hoops from the recipe's path).
    outboard (F.11.2.1, keep-out): every tube and hoop surface clears the box.
    sideProtection (F.11.2.1-.3, R50/R51): in side view the box lies inside the panels of
      Size B side structure, i.e. the polygon BAT0-MH0-CS0-CSU-MHU-BR (battery panel plus the
      side impact panel the box front reaches into at the starter position).
    insidePrimaryStructure (F.11.1.1, R49): every box vertex inside the frame's convex hull."""
    lo, hi = np.array(box['lo']), np.array(box['hi'])
    worst = (np.inf, None)
    for m in rec['members']:
        d = _seg_box_dist(np.array(m['a']), np.array(m['b']), lo, hi) - outer_radius(m)
        if d < worst[0]: worst = (d, m['name'])
    for name, poly in bm.hoops(bm.main_hoop_path(rec)).items():
        for a, b in zip(poly, poly[1:]):
            d = _seg_box_dist(a, b, lo, hi) - HOOP_R
            if d < worst[0]: worst = (d, name)
    n = rec['nodes']; side_ok = True; side_gap = np.inf
    rect = np.array([[x, z] for x in np.linspace(lo[0], hi[0], 60) for z in (lo[2], hi[2])] +
                    [[x, z] for x in (lo[0], hi[0]) for z in np.linspace(lo[2], hi[2], 30)])
    for s in ('L', 'R'):
        poly = np.array([[n[k + s][0], n[k + s][2]] for k in ('BAT0', 'MH0', 'CS0', 'CSU', 'MHU', 'BR')])
        side_ok &= bool(_inside_polygon(rect, poly).all())
        # x gap from the aft battery post (BAT0-BR centreline) to the box's rear face, at its bottom and top
        (xa, za), (xb, zb) = poly[0], poly[5]
        side_gap = min(side_gap, min(lo[0] - (xa + (z - za) * (xb - xa) / (zb - za)) for z in (lo[2], hi[2])))
    pts = [np.array(m[k]) for m in rec['members'] for k in ('a', 'b')]
    pts += [p for poly in bm.hoops(bm.main_hoop_path(rec)).values() for p in poly]
    inside = bool((Delaunay(np.array(pts)).find_simplex(box_vertices(box)) >= 0).all())
    return {'clearanceMm': worst[0] * 1000, 'closest': worst[1], 'outboard': worst[0] >= 0,
            'sideProtection': side_ok, 'boxRearToBatteryPostMm': side_gap * 1000,
            'insidePrimaryStructure': inside, 'pass': worst[0] >= 0 and side_ok and inside}
