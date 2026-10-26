"""Name-free predicates for the rule structure the optimizer may re-route (FSAE 2027 V1.0).

  F.6.4   Side Impact Structure, each side: a Size B upper member joining the hoops with its top
          edge entirely 290-345 mm above the lowest bottom of a Size B lower member that joins
          the hoop feet, and Size B diagonals that join them and completely triangulate every bay
          between the hoops (F.6.4.4-.6, F.1.17).
  F.6.2.3 Front Bulkhead Support, each side: Size C upper, lower and diagonal members from the
          front hoop to the front bulkhead. The upper one meets the bulkhead with its top within
          50 mm of the bulkhead's top surface and the hoop no more than 50 mm below (and, without
          extra bracing, at most 100 mm above) the top of the upper side impact member; the lower
          one joins the hoop base to the bulkhead base; diagonals triangulate every bay.
  F.5.2.3 Every kink of those multi-piece members has a support tube ending there with the
          member's diameter and wall, within 30 deg of the member's plane. The upper side impact
          member only needs a structural tube ending at its kink (F.5.2.3 exception).

Every decision uses tube end coordinates, profiles and connectivity. Tube names, roles and the
recipe's node names are never read to decide; names are only copied into reports.

Triangulation is judged on each panel's side view (x, z), the view F.6.4 and F.6.2.3 are drawn in.
The panel is the polygon bounded by the lower member, the closing post in front (front hoop or
bulkhead side), the upper member and the post behind (main hoop or front hoop). Corners are
measured in 3D, so a member kinked only in plan view (the cockpit side bulge) still has a corner
there. A bay is a triangle when it has exactly three corners: a node partway along one straight
tube, or anywhere along a hoop or the bulkhead side, is not a corner. The panel passes when some
subset of its structural members inside it splits it into triangles; an extra member inside a
triangulated bay does not undo that (F.6.4.6.b asks the diagonals to triangulate the bays).
Tubes below the panel's size class are ignored, as F.3.3 requires.

Known simplifications (all conservative or exact for this frame family): straight tubes only;
the upper bulkhead support must end on the bulkhead side tube (the top corner here); the front
hoop attachment window uses the support's centreline height; members that cross in side view
without a shared joint fail the panel.
"""
from pathlib import Path
import functools, math, sys
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import beam_model as bm

# F.3.4.1 minimums: I mm4, A mm2, OD mm, wall mm (ground_structure.CLASS uses this table)
CLASS = {'A': (11320, 173, 25.0, 2.0), 'B': (8509, 114, 25.0, 1.2), 'C': (6695, 91, 25.0, 1.2)}
RANK = {None: -1, 'C': 0, 'B': 1, 'A': 2}
MERGE = 0.0015                            # tube ends closer than this are one node (m)
KINK_DEG = 1.0                            # direction change that makes a node a corner (F.1.17) or a kink (F.5.2.3)
Y_MIN = 0.05                              # both ends further than this from the centre plane: that side's tube (m)
POST_TOL = 0.0127 + bm.JOINT_CLEARANCE    # a node on a hoop or the bulkhead side: inside its surface + 0.05 in
FOOT_TOL = 0.0127                         # a node at a hoop foot or bulkhead corner
ZONE = (0.290, 0.345)                     # F.6.4.4.b, above the lowest bottom of the lower side impact member
BULKHEAD_TOP = 0.050                      # F.6.2.3.a
HOOP_BELOW, HOOP_ABOVE = 0.050, 0.100     # F.6.2.3.a / .b, about the top of the upper side impact member
PLANE_DEG = 30.0                          # F.5.2.3.c
MAX_PATHS = 400
MAX_PAIRS = 16                            # member paths tried per side and panel (simplest first; fewer can only fail)


@functools.lru_cache(maxsize=None)
def section(profile):
    return bm.section(profile)


@functools.lru_cache(maxsize=None)
def meets(profile, cls):
    s = section(profile); i, a, od, w = CLASS[cls]
    return s['I'] * 1e12 >= i and s['A'] * 1e6 >= a and s['od'] * 1e3 >= od - 1e-9 and s['wall'] * 1e3 >= w - 1e-9


@functools.lru_cache(maxsize=None)
def level(profile):
    """Best F.3.2 size class the profile meets, or None for Non-Structural tubing (F.3.3)."""
    return next((c for c in ('A', 'B', 'C') if meets(profile, c)), None)


@functools.lru_cache(maxsize=4)
def context(main_hoop):
    """Supplied geometry the predicates measure against: both hoop centrelines and the front
    bulkhead rectangle (from the starter's Front Bulkhead Sketch)."""
    from frame_geometry import source_geometry
    h = bm.hoops(main_hoop)
    src, _ = source_geometry()
    q = np.array([s[k] for s in src['Front Bulkhead Sketch'] for k in ('start', 'end')])
    bh = {'x': float(np.mean(q[:, 0])), 'y': float(np.max(np.abs(q[:, 1]))),
          'z0': float(q[:, 2].min()), 'z1': float(q[:, 2].max())}
    return {'main': np.array(h['MainHoop_1']), 'front': np.array(h['FrontHoop_1']), 'bulkhead': bh,
            'mainHoop': main_hoop}


def region(ctx, side):
    """Box that holds every tube either panel on this side can use (search relevance)."""
    m = ctx['main']; leg = m[(m[:, 1] * side > 0) & (m[:, 2] < 0.6)]
    return {'x0': float(leg[:, 0].min()) - 0.03, 'x1': ctx['bulkhead']['x'] + 0.03, 'z1': ctx['bulkhead']['z1'] + 0.06}


def in_region(a, b, side, reg):
    return (a[1] * side > Y_MIN and b[1] * side > Y_MIN and reg['x0'] <= min(a[0], b[0])
            and max(a[0], b[0]) <= reg['x1'] and max(a[2], b[2]) <= reg['z1'])


# ------------------------------------------------------------------ geometry helpers
def _unit(v):
    return v / np.linalg.norm(v)


def _turn(d1, d2):
    c = float(np.clip(_unit(d1) @ _unit(d2), -1, 1)); return math.degrees(math.acos(c))


def _poly_proj(poly, P):
    """Distance from each point of P to the polyline and arc-length position of the nearest point."""
    a, b = poly[:-1], poly[1:]; d = b - a; L2 = np.maximum(np.einsum('ij,ij->i', d, d), 1e-18)
    t = np.clip(np.einsum('nsj,sj->ns', P[:, None, :] - a[None], d) / L2, 0, 1)
    q = a[None] + t[..., None] * d[None]; dist = np.linalg.norm(q - P[:, None, :], axis=2)
    k = np.argmin(dist, axis=1); s0 = np.concatenate([[0.0], np.cumsum(np.sqrt(L2))])
    return dist[np.arange(len(P)), k], s0[k] + t[np.arange(len(P)), k] * np.sqrt(L2[k])


def _seg_dist(P, a, b):
    d = b - a; t = np.clip((P - a) @ d / (d @ d), 0, 1)
    return np.linalg.norm(P - (a + t[:, None] * d), axis=1)


def _cross2(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _proper_cross(p, q, r, s, eps=1e-9):
    """2D segments pq and rs cross at a point interior to both."""
    d1, d2 = _cross2(r, s, p), _cross2(r, s, q); d3, d4 = _cross2(p, q, r), _cross2(p, q, s)
    return ((d1 > eps and d2 < -eps) or (d1 < -eps and d2 > eps)) and ((d3 > eps and d4 < -eps) or (d3 < -eps and d4 > eps))


def _area(poly):
    x, z = poly[:, 0], poly[:, 1]
    return 0.5 * float(np.dot(x, np.roll(z, -1)) - np.dot(z, np.roll(x, -1)))


def _inside(poly, p, margin=5e-4):
    """Point strictly inside a simple 2D polygon, at least `margin` from its boundary."""
    n = len(poly); c = False
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        d = b - a; t = np.clip((p - a) @ d / max(d @ d, 1e-18), 0, 1)
        if np.linalg.norm(p - (a + t * d)) < margin: return False
        if (a[1] > p[1]) != (b[1] > p[1]) and p[0] < a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]):
            c = not c
    return c


# ------------------------------------------------------------------ tube graph
class _Graph:
    """Nodes: merged tube ends. Each tube is split at every node lying inside its outer surface,
    so T-joints and X-brace crossings are nodes like any other."""
    def __init__(self, ms, idx):
        self.idx = list(idx)                     # indices into the caller's member list
        E = np.array([ms[k]['a'] for k in idx] + [ms[k]['b'] for k in idx]).reshape(-1, 3)
        lab = -np.ones(len(E), int); P = []
        if len(E):
            D = np.linalg.norm(E[:, None] - E[None], axis=2)
            for i in range(len(E)):
                if lab[i] < 0:
                    grp = np.where((D[i] < MERGE) & (lab < 0))[0]; lab[grp] = len(P); P.append(E[grp].mean(0))
        self.P = np.array(P).reshape(-1, 3); n = len(idx)
        self.adj = {i: {} for i in range(len(self.P))}; self.chain = []
        for j, k in enumerate(idx):
            i0, i1 = int(lab[j]), int(lab[j + n]); a, b = self.P[i0], self.P[i1]
            d = b - a; L = float(np.linalg.norm(d))
            if L < MERGE: self.chain.append([i0]); continue
            u = d / L; t = (self.P - a) @ u; off = np.linalg.norm(self.P - (a + t[:, None] * u), axis=1)
            r = section(ms[k]['profile'])['od'] / 2
            on = np.where((t > MERGE) & (t < L - MERGE) & (off <= r))[0]
            ch = [i0] + [int(x) for x in on[np.argsort(t[on])] if x not in (i0, i1)] + [i1]
            self.chain.append(ch)
            for p, q in zip(ch, ch[1:]):
                if q not in self.adj[p]: self.adj[p][q] = k; self.adj[q][p] = k


def _paths(G, starts, ends, blocked, low=True):
    """Simple paths along tubes from a start node to an end node with x strictly increasing;
    intermediate nodes may not lie on a post (hoop or bulkhead side). The MAX_PAIRS simplest are
    returned: fewest pieces, then lowest (low=True) or highest mean height."""
    out = []
    def dfs(n, path):
        if len(out) >= MAX_PATHS: return
        for m in G.adj[n]:
            if G.P[m][0] <= G.P[n][0] + 1e-4 or m in path: continue
            if m in ends: out.append(path + [m]); continue
            if m in blocked: continue
            dfs(m, path + [m])
    for s in starts: dfs(s, [s])
    out.sort(key=lambda p: (len(p), (1 if low else -1) * float(np.mean(G.P[p][:, 2]))))
    return out[:MAX_PAIRS]


# ------------------------------------------------------------------ triangulation of one panel
def _triangulate(G, lower, upper, back, front):
    """back/front: (kind, pos, nodes on that post). Returns (solution chords, faces) or a reason."""
    P3, P2 = G.P, G.P[:, [0, 2]]
    def between(post, a, b):
        _, pos, on = post; lo, hi = sorted((pos[a], pos[b]))
        sel = [n for n in on if n not in (a, b) and lo + 1e-6 < pos[n] < hi - 1e-6]
        return sorted(sel, key=lambda n: abs(pos[n] - pos[a]))
    fb = between(front, lower[-1], upper[-1]); bb = between(back, upper[0], lower[0])
    if set(fb) & set(lower + upper) or set(bb) & set(lower + upper): return 'post node on a member path'
    cyc = lower + fb + upper[::-1] + bb
    if len(set(cyc)) != len(cyc): return 'upper and lower members share a node'
    kind = {}
    for path in (lower, upper):
        for p, q in zip(path, path[1:]): kind[(p, q)] = kind[(q, p)] = 'member'
    for chain_, k in (([lower[-1]] + fb + [upper[-1]], front[0]), ([upper[0]] + bb + [lower[0]], back[0])):
        for p, q in zip(chain_, chain_[1:]): kind[(p, q)] = kind[(q, p)] = k
    poly = P2[cyc]
    if _area(poly) <= 1e-6: return 'upper member not above the lower member in side view'
    nb = len(cyc)
    for i in range(nb):
        for j in range(i + 2, nb):
            if i == 0 and j == nb - 1: continue
            if _proper_cross(poly[i], poly[(i + 1) % nb], poly[j], poly[(j + 1) % nb]): return 'panel outline crosses itself'
    B = set(cyc)
    # Members inside the panel: tubes whose ends are both on the outline, then tubes ending on
    # the interior nodes those create (X-brace halves, T-joints on a diagonal).
    inner, pieces = set(), set()
    for _ in range(3):
        grew = False
        for ch in G.chain:
            if len(ch) < 2 or not (ch[0] in B | inner and ch[-1] in B | inner): continue
            if any(n in B for n in ch[1:-1]):
                continue                                    # runs along the outline
            for p, q in zip(ch, ch[1:]):
                e = (min(p, q), max(p, q))
                if kind.get((p, q)) or e in pieces: continue
                pieces.add(e); grew = True
            for n in ch[1:-1]:
                if n not in inner: inner.add(n); grew = True
        if not grew: break
    pieces = {e for e in pieces if _inside(poly, (P2[e[0]] + P2[e[1]]) / 2)}
    inner = {n for n in inner if _inside(poly, P2[n]) and any(n in e for e in pieces)}
    pieces = {e for e in pieces if all(n in B or n in inner for n in e)}
    pl = sorted(pieces)
    for i, (p, q) in enumerate(pl):
        for r, s in pl[i + 1:]:
            if {p, q} & {r, s}: continue
            if _proper_cross(P2[p], P2[q], P2[r], P2[s]): return 'members cross in the panel without a joint'
        for i0 in range(nb):
            r, s = cyc[i0], cyc[(i0 + 1) % nb]
            if {p, q} & {r, s}: continue
            if _proper_cross(P2[p], P2[q], P2[r], P2[s]): return 'a member leaves the panel'
    adjc = {}
    for p, q in pieces:
        adjc.setdefault(p, set()).add(q); adjc.setdefault(q, set()).add(p)
        kind[(p, q)] = kind[(q, p)] = 'chord'
    xz = {n: (float(P2[n][0]), float(P2[n][1])) for n in B | inner}
    cos_kink = math.cos(math.radians(KINK_DEG)); turn = {}
    def corner(u, v, w):
        k = (u, v, w)
        if k not in turn:
            k1, k2 = kind[(u, v)], kind[(v, w)]
            if k1 == k2 and k1.startswith('post'): turn[k] = False
            else:
                d1, d2 = P3[v] - P3[u], P3[w] - P3[v]
                turn[k] = float(d1 @ d2) < cos_kink * float(np.linalg.norm(d1) * np.linalg.norm(d2))
        return turn[k]
    def area(face):
        return 0.5 * sum(xz[a][0] * xz[b][1] - xz[b][0] * xz[a][1] for a, b in zip(face, face[1:] + face[:1]))
    def into(face, i, w):
        """Edge from face[i] towards w starts inside the (CCW) face: inside the corner's angle."""
        u = xz[face[i]]; nx = xz[face[(i + 1) % len(face)]]; pv = xz[face[i - 1]]; d = xz[w]
        a_n = math.atan2(nx[1] - u[1], nx[0] - u[0])
        span = (math.atan2(pv[1] - u[1], pv[0] - u[0]) - a_n) % (2 * math.pi)
        rel = (math.atan2(d[1] - u[1], d[0] - u[0]) - a_n) % (2 * math.pi)
        return 1e-6 < rel < span - 1e-6
    memo = {}
    def solve(face):
        r0 = face.index(min(face)); key = tuple(face[r0:] + face[:r0])
        if key in memo: return memo[key]
        memo[key] = None
        if area(face) <= 1e-7: return None
        nf = len(face)
        nc = sum(corner(face[i - 1], face[i], face[(i + 1) % nf]) for i in range(nf))
        if nc == 3: memo[key] = ([], [list(face)]); return memo[key]
        if nc < 3: return None
        fset = set(face); pos = {n: i for i, n in enumerate(face)}
        for u in face:
            stack = [[u]]
            while stack:
                path = stack.pop(); n = path[-1]
                for w in adjc.get(n, ()):
                    if w in path: continue
                    if len(path) == 1:
                        if w in pos and abs(pos[u] - pos[w]) in (1, nf - 1): continue   # a face edge
                        if not into(face, pos[u], w): continue      # leaves through another face
                    if w in fset:
                        if pos[w] <= pos[u]: continue
                        a, b = pos[u], pos[w]; inner_ = path[1:]
                        f1 = face[a:b + 1] + inner_[::-1]; f2 = face[b:] + face[:a + 1] + inner_
                        s1 = solve(f1)
                        if s1 is None: continue
                        s2 = solve(f2)
                        if s2 is None: continue
                        memo[key] = (s1[0] + s2[0] + [path + [w]], s1[1] + s2[1]); return memo[key]
                    elif len(path) < 4:
                        stack.append(path + [w])
        return None
    sol = solve(list(cyc))
    if sol is None:
        # Split the panel by every member inside it; the bays left with more than three corners are
        # what a further diagonal must triangulate (reported so a search can repair them).
        def finest(face):
            nf = len(face); fset = set(face); pos = {n: i for i, n in enumerate(face)}
            for i, u in enumerate(face):
                for w in adjc.get(u, ()):
                    if w in pos and abs(pos[u] - pos[w]) in (1, nf - 1): continue
                    if not into(face, i, w): continue
                    path = [u, w]
                    while path[-1] not in fset and len(path) < 5:   # through interior nodes
                        nxt = [x for x in adjc.get(path[-1], ()) if x not in path]
                        if not nxt: break
                        path.append(nxt[0])
                    if path[-1] not in fset: continue
                    a, b = sorted((pos[path[0]], pos[path[-1]]))
                    inner_ = path[1:-1] if pos[path[0]] == a else path[-2:0:-1]
                    return finest(face[a:b + 1] + inner_[::-1]) + finest(face[b:] + face[:a + 1] + inner_)
            nc = sum(corner(face[i - 1], face[i], face[(i + 1) % nf]) for i in range(nf))
            return [face] if nc > 3 else []
        return {'fail': 'bays not completely triangulated', 'faces': finest(list(cyc))}
    return {'cycle': cyc, 'chords': sol[0], 'faces': sol[1], 'kind': kind}


def _kinks(G, path, exempt):
    out = []
    for i in range(1, len(path) - 1):
        u, v, w = path[i - 1], path[i], path[i + 1]
        d1, d2 = G.P[v] - G.P[u], G.P[w] - G.P[v]
        if _turn(d1, d2) > KINK_DEG:
            out.append({'node': G.P[v].tolist(), 'normal': _unit(np.cross(d1, d2)).tolist(),
                        'members': [G.adj[u][v], G.adj[v][w]], 'exempt': exempt})
    return out


def _structure(G, lower, upper, tri, upper_exempt):
    """Members used (indices into the caller's list) and the F.5.2.3 kinks to support."""
    U, L = set(upper), set(lower)
    reach, frontier = set(U), list(U)
    pieces = [(p[i], p[i + 1]) for p in tri['chords'] for i in range(len(p) - 1)]
    nbr = {}
    for p, q in pieces: nbr.setdefault(p, set()).add(q); nbr.setdefault(q, set()).add(p)
    while frontier:
        n = frontier.pop()
        for m in nbr.get(n, ()):
            if m not in reach: reach.add(m); frontier.append(m)
    diagonal = bool(reach & L)
    used_chords = sorted({G.adj[p][q] for p, q in pieces})
    return {'upper': [G.adj[p][q] for p, q in zip(upper, upper[1:])],
            'lower': [G.adj[p][q] for p, q in zip(lower, lower[1:])],
            'diagonals': used_chords, 'bays': len(tri['faces']), 'diagonalJoinsMembers': diagonal,
            'kinks': _kinks(G, lower, False) + _kinks(G, upper, upper_exempt)}


def kink_ok(k, ms):
    """F.5.2.3 support at one kink: a tube (not the member's own pieces) ending at the kink with the
    member's OD and wall, within 30 deg of the member's plane; for the upper side impact member
    any structural tube ending there."""
    p = np.asarray(k['node']); nrm = np.asarray(k['normal'])
    s0, s1 = (section(ms[i]['profile']) for i in k['members'])
    same = abs(s0['od'] - s1['od']) < 1e-9 and abs(s0['wall'] - s1['wall']) < 1e-9
    for j, m in enumerate(ms):
        if j in k['members']: continue
        a, b = np.asarray(m['a']), np.asarray(m['b'])
        for e, o in ((a, b), (b, a)):
            if np.linalg.norm(e - p) > bm.JOINT_CLEARANCE: continue
            if k['exempt']:
                if level(m['profile']): return True
                continue
            s = section(m['profile'])
            if same and abs(s['od'] - s0['od']) < 1e-9 and abs(s['wall'] - s0['wall']) < 1e-9:
                if math.degrees(math.asin(min(1.0, abs(_unit(o - e) @ nrm)))) <= PLANE_DEG: return True
    return False


# ------------------------------------------------------------------ the two panels
def _post_nodes(G, poly, side):
    d, s = _poly_proj(poly, G.P) if len(G.P) else (np.zeros(0), np.zeros(0))
    on = [n for n in range(len(G.P)) if d[n] <= POST_TOL and G.P[n][1] * side > 0]
    return on, s


def _foot(poly, side):
    return poly[0] if poly[0][1] * side > 0 else poly[-1]


def side_impact(ms, side, ctx):
    """F.6.4 on one side. Returns {'pass', 'reason', 'structures', 'zone'}; structures pass every
    check except the F.5.2.3 kink supports (kink_ok), which the caller applies."""
    fx = float(ctx['front'][:, 0].max()); reg = region(ctx, side)
    idx = [k for k, m in enumerate(ms) if meets(m['profile'], 'B') and in_region(m['a'], m['b'], side, reg)
           and max(m['a'][0], m['b'][0]) <= fx + 0.03]
    G = _Graph(ms, idx)
    out = {'pass': False, 'reason': None, 'structures': [], 'zone': None, 'usimTops': [], 'deficient': []}
    if not len(G.P): out['reason'] = 'no Size B tubes between the hoops'; return out
    on_m, pos_m = _post_nodes(G, ctx['main'], side); on_f, pos_f = _post_nodes(G, ctx['front'], side)
    fm, ff = _foot(ctx['main'], side), _foot(ctx['front'], side)
    blocked = set(on_m) | set(on_f)
    lo_s = [n for n in on_m if np.linalg.norm(G.P[n] - fm) <= FOOT_TOL]
    lo_e = {n for n in on_f if np.linalg.norm(G.P[n] - ff) <= FOOT_TOL}
    up_s = [n for n in on_m if G.P[n][2] > fm[2] + 0.10]
    up_e = {n for n in on_f if G.P[n][2] > ff[2] + 0.10}
    lowers, uppers = _paths(G, lo_s, lo_e, blocked), _paths(G, up_s, up_e, blocked, low=False)
    if not lowers: out['reason'] = 'no Size B lower member joining the hoop feet'; return out
    if not uppers: out['reason'] = 'no Size B upper member joining the hoops'; return out
    r = lambda p, q: section(ms[G.adj[p][q]]['profile'])['od'] / 2
    best_zone, reasons = None, []
    for lo in lowers:
        zlb = min(min(G.P[p][2], G.P[q][2]) - r(p, q) for p, q in zip(lo, lo[1:]))
        for up in uppers:
            tops = [G.P[n][2] + r(p, q) - zlb for p, q in zip(up, up[1:]) for n in (p, q)]
            ok = min(tops) >= ZONE[0] - 1e-9 and max(tops) <= ZONE[1] + 1e-9
            margin = min(min(tops) - ZONE[0], ZONE[1] - max(tops))
            if best_zone is None or margin > best_zone['marginM']:
                best_zone = {'ok': ok, 'topsAboveLowerBottomM': [round(float(t), 4) for t in tops], 'marginM': round(float(margin), 4)}
            if not ok: reasons.append('upper member top edge outside 290-345 mm'); continue
            out['usimTops'].append(G.P[up[-1]][2] + r(up[-2], up[-1]))
            tri = _triangulate(G, lo, up, ('post:main', pos_m, on_m), ('post:front', pos_f, on_f))
            if isinstance(tri, str): reasons.append(tri); continue
            if 'fail' in tri:
                reasons.append(tri['fail']); out['deficient'] += [[G.P[n].tolist() for n in f] for f in tri['faces']]; continue
            S = _structure(G, lo, up, tri, upper_exempt=True)
            if not S['diagonalJoinsMembers']: reasons.append('no diagonal joins the upper and lower members'); continue
            S['zone'] = {'topsAboveLowerBottomM': [round(float(t), 4) for t in tops], 'lowestBottomM': round(float(zlb), 4)}
            S['usimTopAtFrontHoopM'] = float(G.P[up[-1]][2] + r(up[-2], up[-1]))
            out['structures'].append(S)
    out['zone'] = best_zone; out['pass'] = bool(out['structures'])
    if not out['pass']: out['reason'] = max(set(reasons), key=reasons.count) if reasons else 'no structure'
    return out


def bulkhead_support(ms, side, ctx, usim_top):
    """F.6.2.3 on one side; usim_top: height of the top of the upper side impact member at the
    front hoop (m). Same return format as side_impact."""
    bh = ctx['bulkhead']; fx = float(ctx['front'][:, 0].min())
    out = {'pass': False, 'reason': None, 'structures': [], 'attach': None, 'deficient': []}
    if usim_top is None: out['reason'] = 'no upper side impact member to support'; return out
    idx = [k for k, m in enumerate(ms) if meets(m['profile'], 'C') and in_region(m['a'], m['b'], side, region(ctx, side))
           and fx - 0.03 <= min(m['a'][0], m['b'][0])]
    G = _Graph(ms, idx)
    if not len(G.P): out['reason'] = 'no Size C tubes between the front hoop and the bulkhead'; return out
    on_f, pos_f = _post_nodes(G, ctx['front'], side)
    A, Bt = np.array([bh['x'], side * bh['y'], bh['z0']]), np.array([bh['x'], side * bh['y'], bh['z1']])
    d_bh = _seg_dist(G.P, A, Bt)
    on_b = [n for n in range(len(G.P)) if d_bh[n] <= POST_TOL]; pos_b = G.P[:, 2]
    # The bulkhead side tube closes the panel; its top tube sets the top surface (F.6.2.3.a).
    def along(m, p, q):
        return np.linalg.norm(np.cross(np.asarray(m['a']) - p, q - p)) / np.linalg.norm(q - p) <= MERGE and \
               np.linalg.norm(np.cross(np.asarray(m['b']) - p, q - p)) / np.linalg.norm(q - p) <= MERGE and \
               abs(m['a'][0] - bh['x']) <= MERGE and abs(m['b'][0] - bh['x']) <= MERGE
    Ct = np.array([bh['x'], -side * bh['y'], bh['z1']])
    if not any(along(ms[k], A, Bt) for k in idx): out['reason'] = 'no structural tube along the bulkhead side'; return out
    tops = [section(m['profile'])['od'] / 2 for m in ms if level(m['profile']) and along(m, Bt, Ct)]
    top_surface = bh['z1'] + (max(tops) if tops else 0.0127)
    ff = _foot(ctx['front'], side); blocked = set(on_f) | set(on_b)
    lo_s = [n for n in on_f if np.linalg.norm(G.P[n] - ff) <= FOOT_TOL]
    lo_e = {n for n in on_b if np.linalg.norm(G.P[n] - A) <= FOOT_TOL}
    up_s = [n for n in on_f if usim_top - HOOP_BELOW - 1e-9 <= G.P[n][2] <= usim_top + HOOP_ABOVE + 1e-9]
    up_e = set(on_b)
    lowers, uppers = _paths(G, lo_s, lo_e, blocked), _paths(G, up_s, up_e, blocked, low=False)
    out['attach'] = {'usimTopAtFrontHoopM': round(float(usim_top), 4), 'bulkheadTopSurfaceM': round(float(top_surface), 4)}
    if not lowers: out['reason'] = 'no Size C lower support from the front hoop base to the bulkhead base'; return out
    r = lambda p, q: section(ms[G.adj[p][q]]['profile'])['od'] / 2
    uppers = [u for u in uppers if abs(top_surface - (G.P[u[-1]][2] + r(u[-2], u[-1]))) <= BULKHEAD_TOP + 1e-9]
    if not uppers: out['reason'] = 'no Size C upper support meeting the F.6.2.3.a attachment heights'; return out
    reasons = []
    for lo in lowers:
        for up in uppers:
            tri = _triangulate(G, lo, up, ('post:front', pos_f, on_f), ('post:bulkhead', pos_b, on_b))
            if isinstance(tri, str): reasons.append(tri); continue
            if 'fail' in tri:
                reasons.append(tri['fail']); out['deficient'] += [[G.P[n].tolist() for n in f] for f in tri['faces']]; continue
            S = _structure(G, lo, up, tri, upper_exempt=False)
            if not S['diagonalJoinsMembers']: reasons.append('no diagonal joins the upper and lower supports'); continue
            S['attach'] = {'frontHoopZM': round(float(G.P[up[0]][2]), 4),
                           'belowUsimTopM': round(float(usim_top - G.P[up[0]][2]), 4),
                           'belowBulkheadTopM': round(float(top_surface - G.P[up[-1]][2] - r(up[-2], up[-1])), 4)}
            out['structures'].append(S)
    out['pass'] = bool(out['structures'])
    if not out['pass']: out['reason'] = max(set(reasons), key=reasons.count) if reasons else 'no structure'
    return out


def panels(ms, side, ctx):
    """Both panels on one side, kink supports not yet applied (see kink_ok)."""
    si = side_impact(ms, side, ctx)
    # F.6.2.3.a/b measure from the top of the upper side impact member at the front hoop: the passing
    # side structures' upper member, else any upper member in the F.6.4.4 zone.
    tops = [S['usimTopAtFrontHoopM'] for S in si['structures']] or si['usimTops']
    return {'sideImpact': si, 'bulkheadSupport': bulkhead_support(ms, side, ctx, max(tops) if tops else None)}


def _members(recipe):
    return [{'a': np.asarray(m['a'], float), 'b': np.asarray(m['b'], float), 'profile': m['profile'],
             'name': m.get('name')} for m in recipe['members']]


def check(recipe):
    """Full F.6.4 / F.6.2.3 verdict for a recipe, kink supports included."""
    ms = _members(recipe); ctx = context(bm.main_hoop_path(recipe)); out = {'sideImpact': {}, 'bulkheadSupport': {}}
    name = lambda ks: [ms[k]['name'] for k in ks]
    for side, lab in ((1, 'L'), (-1, 'R')):
        res = panels(ms, side, ctx)
        for key, rule in (('sideImpact', 'F.6.4'), ('bulkheadSupport', 'F.6.2.3')):
            r = res[key]; good = [S for S in r['structures'] if all(kink_ok(k, ms) for k in S['kinks'])]
            rep = {'pass': bool(good), 'rule': rule}
            if good:
                S = good[0]
                rep.update(upper=name(S['upper']), lower=name(S['lower']), diagonals=name(S['diagonals']), bays=S['bays'],
                           kinkSupports=len(S['kinks']), **({'zone': S['zone']} if 'zone' in S else {}),
                           **({'attach': S['attach']} if 'attach' in S else {}))
            else:
                rep['reason'] = r['reason'] if not r['structures'] else 'a kink of the upper or lower member has no F.5.2.3 support'
            if key == 'sideImpact': rep['zoneOk'] = bool(r['zone'] and r['zone']['ok']); rep['bestZone'] = r['zone']
            else: rep['attachHeights'] = r.get('attach')
            out[key][lab] = rep
    return out
