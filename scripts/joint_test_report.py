"""Per-case joint result for a joint-diagnostics test frame (carrier C<k> + coped branch B<k>).

  python scripts/joint_test_report.py <test dir> <build evidence dir> <joint-diagnostics evidence dir> [clearance mm]

Writes <test dir>/joint-results-<clearance>mm.json. Diagnostic only.
"""
import json, math, re, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import joint_connectivity as jc

d, build_ev, diag_ev = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
c = sys.argv[4] if len(sys.argv) > 4 else '1.27'
res = json.load(open(diag_ev / 'stdout.json', encoding='utf-8-sig'))['result']
names = {b['index']: re.sub(r'^Beam-\d+\((.*)\)$', r'\1', b['name']) for b in res['beams']}
inv = {v: k for k, v in names.items()}
build = json.load(open(build_ev / 'stdout.json', encoding='utf-8-sig'))['result']
measured = {x['target']: x.get('distanceM') for x in build['cuts']}
rec = json.load(open(d / 'joint-test-recipe.json'))
deck = d / f'study-test/deck-{c}mm/audit.DAT'
G, bars, rb, rad = jc.read_deck(deck)
comps = jc.analyse(deck, names)['components']
rows = []
for case in rec['cases']:
    k = case['case']; p = inv[case['branch']]
    ns = {n for q, a, b in bars if q == p for n in (a, b)}
    end = min(math.dist(G[n], [0.5 * k, 0, 0]) for n in ns)
    joined = any(case['branch'] in x and case['carrier'] in x for x in comps)
    gap = measured.get(case['branch'])
    rows.append({**case, 'measuredGapMm': None if gap is None else gap * 1000, 'branchEndFromJointMm': end * 1000, 'joined': joined})
    shown = 'uncoped' if gap is None else f'{gap * 1000:.4f}'
    print(f"case{k:2d} {case['branchProfile'][:5]} on {case['carrierProfile'][:5]} {case['angleDeg']:5.1f} deg  cope gap "
          f"{case['gapM'] * 1000:5.3f} mm (measured {shown})  beam end {end * 1000:6.2f} mm from joint  {'JOINED' if joined else 'NOT JOINED'}")
json.dump(rows, open(d / f'joint-results-{c}mm.json', 'w'), indent=1)
