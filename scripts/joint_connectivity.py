"""Read beam connectivity from a native Nastran audit deck (joint diagnostics).

  python scripts/joint_connectivity.py <audit.DAT> <stdout.json of the native job> [--pairs recipe.json]

Prints connected components by beam name and, for each free beam end, the nearest other
beam and the distance from that end to its axis. Diagnostic only; no stiffness is computed.
"""
import json, math, re, sys
from collections import defaultdict


def num(s):
    s = s.strip()
    if not s: return 0.0
    if re.search('[eEdD]', s): return float(s.replace('D', 'E'))
    return float(re.sub(r'(?<=[0-9.])([+-]\d+)$', r'e\1', s))


def read_deck(path):
    lines = open(path).read().splitlines()
    grid, bars, rbars, radius = {}, [], [], {}
    for i, l in enumerate(lines):
        f = [x.strip() for x in l.split(',')]
        if f[0] == 'GRID': grid[int(f[1])] = [num(x) for x in f[3:6]]
        elif f[0] == 'CBAR': bars.append((int(f[2]), int(f[3]), int(f[4])))
        elif f[0] == 'RBAR': rbars.append((int(f[2]), int(f[3])))
        elif f[0] == 'PBAR': radius[int(f[1])] = num(lines[i + 1].split(',')[3])
    return grid, bars, rbars, radius


def analyse(deck, names):
    grid, bars, rbars, radius = read_deck(deck)
    parent = {}
    def find(x):
        while parent.setdefault(x, x) != x: x = parent[x]
        return x
    def union(a, b): parent[find(a)] = find(b)
    for p, a, b in bars: union(a, b)
    for a, b in rbars: union(a, b)
    comps = defaultdict(set)
    for p, a, b in bars: comps[find(a)].add(names.get(p, f'Beam-{p}'))
    deg = defaultdict(int); own = defaultdict(lambda: defaultdict(int))
    for p, a, b in bars:
        deg[a] += 1; deg[b] += 1; own[p][a] += 1; own[p][b] += 1
    for a, b in rbars: deg[a] += 1; deg[b] += 1
    def segd(q, a, b):
        ab = [y - x for x, y in zip(a, b)]; t = sum((q[i] - a[i]) * ab[i] for i in range(3)) / sum(x * x for x in ab)
        t = max(0, min(1, t)); return math.dist(q, [a[i] + t * ab[i] for i in range(3)])
    free = []
    for p, nodes in own.items():
        for n, k in nodes.items():
            if k == 1 and deg[n] == 1:
                near = min((segd(grid[n], grid[a], grid[b]), q) for q, a, b in bars if q != p)
                free.append({'beam': names.get(p, p), 'radiusMm': radius[p] * 1000, 'nearest': names.get(near[1], near[1]),
                             'nearestRadiusMm': radius[near[1]] * 1000, 'distanceToAxisMm': near[0] * 1000})
    return {'components': sorted((sorted(c) for c in comps.values()), key=len, reverse=True), 'freeEnds': free,
            'rigidLinks': len(rbars)}


if __name__ == '__main__':
    deck, native = sys.argv[1], sys.argv[2]
    res = json.load(open(native, encoding='utf-8-sig'))['result']
    names = {b['index']: re.sub(r'^Beam-\d+\((.*)\)$', r'\1', b['name']) for b in res['beams']}
    out = analyse(deck, names)
    print(json.dumps(out, indent=1))
