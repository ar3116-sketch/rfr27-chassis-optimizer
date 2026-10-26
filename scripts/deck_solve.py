"""Diagnostic only: independent linear solve of a SOLIDWORKS-exported Nastran beam deck.

Reads the cards the native beam export writes (GRID, CBAR, PBAR, MAT1, RBAR, SPC, FORCE) and
solves K u = f with Timoshenko beams (PBAR K1/K2 shear factors) and rigid RBAR groups, so a deck
can be checked against SOLIDWORKS' own solved rotations. It is not a score authority: scores come
from the native study and scripts/validate-chassis-simulation.py.

  python scripts/deck_solve.py DECK.DAT [--dedupe] x,y,z [x,y,z ...]

Prints RX (deg) at the mesh node nearest each probe point and TR = 545 / mean |RX|.
--dedupe drops repeated CBARs (same property and the same two GRIDs), e.g. the 67 duplicated
elements in the 009c hand-run deck (cad/tests/manual-tr-009c/deck-export/audit.DAT).
"""
import math, re, sys
from collections import defaultdict
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla


def num(v):
    v = v.strip()
    if not v: return 0.0
    if re.search('[eEdD]', v): return float(v.replace('D', 'E').replace('d', 'e'))
    return float(re.sub(r'(?<=[0-9.])([+-]\d+)$', r'e\1', v))


def parse(path):
    L = open(path).read().splitlines()
    grid, cbar, rbar, pbar, spc, force = {}, [], [], {}, defaultdict(set), defaultdict(lambda: np.zeros(3))
    E = G = None
    for i, raw in enumerate(L):
        f = [s.strip() for s in raw.split(',')]
        k = f[0]
        if k == 'GRID': grid[int(f[1])] = np.array([num(x) for x in f[3:6]])
        elif k == 'CBAR': cbar.append((int(f[1]), int(f[2]), int(f[3]), int(f[4]), f[5]))
        elif k == 'RBAR': rbar.append((int(f[2]), int(f[3]), f[4], f[5] if len(f) > 5 else ''))
        elif k == 'PBAR':
            k2 = [s.strip() for s in L[i + 2].split(',')]
            pbar[int(f[1])] = dict(A=num(f[3]), I1=num(f[4]), I2=num(f[5]), J=num(f[6]), K1=num(k2[1]), K2=num(k2[2]))
        elif k == 'MAT1': E, G = num(f[2]), num(f[3])
        elif k == 'SPC': spc[int(f[2])] |= set(f[3])
        elif k == 'FORCE': force[int(f[2])] += num(f[4]) * np.array([num(x) for x in f[5:8]])
    return grid, cbar, rbar, pbar, spc, force, E, G


def k_local(Lr, A, Iy, Iz, J, E, G, ky, kz):
    # Timoshenko 3D beam; Iz bends in local x-y (plane 1), Iy in x-z (plane 2).
    py = 12 * E * Iz / (ky * G * A * Lr * Lr) if ky > 0 else 0.0
    pz = 12 * E * Iy / (kz * G * A * Lr * Lr) if kz > 0 else 0.0
    k = np.zeros((12, 12))
    a = E * A / Lr; t = G * J / Lr
    k[0, 0] = k[6, 6] = a; k[0, 6] = k[6, 0] = -a
    k[3, 3] = k[9, 9] = t; k[3, 9] = k[9, 3] = -t
    # bending in x-y: v (1,7), rz (5,11)
    c = E * Iz / ((1 + py) * Lr ** 3)
    idx = [1, 5, 7, 11]
    m = c * np.array([[12, 6 * Lr, -12, 6 * Lr], [6 * Lr, (4 + py) * Lr * Lr, -6 * Lr, (2 - py) * Lr * Lr],
                      [-12, -6 * Lr, 12, -6 * Lr], [6 * Lr, (2 - py) * Lr * Lr, -6 * Lr, (4 + py) * Lr * Lr]])
    for r in range(4):
        for s in range(4): k[idx[r], idx[s]] += m[r, s]
    # bending in x-z: w (2,8), ry (4,10); sign flips on coupling terms
    c = E * Iy / ((1 + pz) * Lr ** 3)
    idx = [2, 4, 8, 10]
    m = c * np.array([[12, -6 * Lr, -12, -6 * Lr], [-6 * Lr, (4 + pz) * Lr * Lr, 6 * Lr, (2 - pz) * Lr * Lr],
                      [-12, 6 * Lr, 12, 6 * Lr], [-6 * Lr, (2 - pz) * Lr * Lr, 6 * Lr, (4 + pz) * Lr * Lr]])
    for r in range(4):
        for s in range(4): k[idx[r], idx[s]] += m[r, s]
    return k


def solve(path, dedupe=False):
    grid, cbar, rbar, pbar, spc, force, E, G = parse(path)
    if dedupe:
        seen, keep = set(), []
        for c in cbar:
            key = (c[1], tuple(sorted(c[2:4])))
            if key in seen: continue
            seen.add(key); keep.append(c)
        print(f'  dedupe: {len(cbar)} -> {len(keep)} CBAR')
        cbar = keep
    phys = sorted({n for c in cbar for n in c[2:4]} | {n for r in rbar for n in r[:2]})
    idx = {n: i for i, n in enumerate(phys)}; N = 6 * len(phys)
    rows, cols, vals = [], [], []
    for eid, pid, a, b, g0 in cbar:
        p = pbar[pid]; xa, xb = grid[a], grid[b]; d = xb - xa; Lr = np.linalg.norm(d); ex = d / Lr
        v = grid[int(g0)] - xa if re.fullmatch(r'\d+', g0) else np.array([1.0, 0, 0])
        ey = v - v.dot(ex) * ex
        if np.linalg.norm(ey) < 1e-9: ey = np.cross(ex, [0, 0, 1.0]) if abs(ex[2]) < .9 else np.cross(ex, [0, 1.0, 0])
        ey /= np.linalg.norm(ey); ez = np.cross(ex, ey)
        R = np.vstack([ex, ey, ez]); T = np.kron(np.eye(4), R)
        kg = T.T @ k_local(Lr, p['A'], p['I2'], p['I1'], p['J'], E, G, p['K1'], p['K2']) @ T
        dof = [6 * idx[a] + i for i in range(6)] + [6 * idx[b] + i for i in range(6)]
        for r in range(12):
            for s in range(12):
                rows.append(dof[r]); cols.append(dof[s]); vals.append(kg[r, s])
    K = sp.csr_matrix((vals, (rows, cols)), shape=(N, N))
    F = np.zeros(N)
    for n, f in force.items(): F[6 * idx[n]:6 * idx[n] + 3] += f
    # RBAR: GB (dependent, 123456) follows GA rigidly. Build u = T q over independent dofs.
    # Rigid links are symmetric: each rigidly linked group moves as one body about one master
    # node (the lowest id). SOLIDWORKS ties several beam ends to one joint node this way.
    parent = {}
    def find(n):
        while parent.get(n, n) != n: n = parent[n]
        return n
    for a, b, cna, cnb in rbar:
        assert cna == '123456' and cnb == '', (cna, cnb)
        ra, rb = find(a), find(b)
        if ra != rb: parent[max(ra, rb)] = min(ra, rb)
    # Master of each group: its restrained node if it has one, else the lowest id.
    groups = defaultdict(list)
    for n in phys: groups[find(n)].append(n)
    rep = {}
    for r, ns in groups.items():
        held = [n for n in ns if n in spc]
        assert len(held) <= 1, 'two restrained nodes in one rigid group'
        rep[r] = held[0] if held else min(ns)
    dep = {n: rep[find(n)] for n in phys if rep[find(n)] != n}
    def master(n): return dep.get(n, n)
    indep = [n for n in phys if n not in dep]
    qidx = {n: i for i, n in enumerate(indep)}
    Tr, Tc, Tv = [], [], []
    for n in phys:
        if n not in dep:
            for i in range(6): Tr.append(6 * idx[n] + i); Tc.append(6 * qidx[n] + i); Tv.append(1.0)
            continue
        m = master(n); r = grid[n] - grid[m]
        # u_n = u_m + th_m x r ; th_n = th_m
        S = np.array([[0, r[2], -r[1]], [-r[2], 0, r[0]], [r[1], -r[0], 0]])  # th x r = S th
        blk = np.eye(6); blk[0:3, 3:6] = S
        for i in range(6):
            for j in range(6):
                if blk[i, j] != 0: Tr.append(6 * idx[n] + i); Tc.append(6 * qidx[m] + j); Tv.append(blk[i, j])
    T = sp.csr_matrix((Tv, (Tr, Tc)), shape=(N, 6 * len(indep)))
    Kq = (T.T @ K @ T).tocsr(); Fq = T.T @ F
    fixed = set()
    for n, dofs in spc.items():
        if n not in qidx:
            if n in idx: raise ValueError('SPC on dependent node')
            continue
        for ch in dofs: fixed.add(6 * qidx[n] + int(ch) - 1)
    free = np.array([i for i in range(Kq.shape[0]) if i not in fixed])
    # AUTOSPC: drop zero-stiffness dofs
    diag = Kq.diagonal()
    free = np.array([i for i in free if abs(diag[i]) > 1e-9 * abs(diag).max()])
    q = np.zeros(Kq.shape[0])
    q[free] = spla.spsolve(Kq[free][:, free].tocsc(), Fq[free])
    u = T @ q
    rot = {n: u[6 * idx[n] + 3:6 * idx[n] + 6] for n in phys}
    return grid, rot, phys, len(cbar)


if __name__ == '__main__':
    path = sys.argv[1]; dedupe = '--dedupe' in sys.argv
    probes = [[float(x) for x in a.split(',')] for a in sys.argv[2:] if re.fullmatch(r'[-\d.e,]+', a)]
    grid, rot, phys, n = solve(path, dedupe)
    rx = []
    for p in probes:
        d, nn = min((np.linalg.norm(grid[k] - np.array(p)), k) for k in phys)
        rx.append(math.degrees(rot[nn][0])); print(f'  probe {p} node {nn} ({d*1000:.2f} mm) RX {rx[-1]:.5f} deg')
    print(f'  CBAR {n}; TR = {545 / abs(sum(rx) / len(rx)):.1f}')
