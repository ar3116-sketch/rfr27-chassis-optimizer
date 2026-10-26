"""The team's Main Roll Hoop Sketch as a one-piece weldment path (design/main-hoop-source-001.json).

The sketch (design/constraint-ledger.json hoops[0]) is tangent everywhere except one line-line
corner per side at z 626 mm (0.383 deg). A structural member splits there, so the sketch builds as
three hoop bodies, but F.5.6 needs one continuous tube. Each corner gets a tangent arc of 60 in
radius (about 10 mm long; at the sketch's 4.0 in radius the arc is 0.7 mm and SOLIDWORKS refuses to
sketch it, 20 Oct attempt 1); that moves the centreline by under 0.01 mm. Every other segment is
the sketch exactly. Output uses the native repair-main-hoop format (line/arc with mid points).

  python scripts/prepare-source-hoop.py
"""
import json, math
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'design/main-hoop-source-001.json'
R = 60 * 0.0254   # the bend is 0.383 deg: about 10 mm of arc


def main():
    src = json.loads((ROOT / 'design/constraint-ledger.json').read_text(encoding='utf-8-sig'))['hoops'][0]['worldSegments']
    # Chain the unordered segments from the left foot (+Y, lowest z) round to the right foot.
    pieces = []
    for s in src:
        a, b = np.array(s['start']), np.array(s['end'])
        pieces.append({'arc': s['kind'] == 1, 'a': a, 'b': b, 'c': np.array(s['center']) if s['kind'] == 1 else None,
                       'r': s.get('radius'), 'idx': s['sourceIndex']})
    ends = [p for q in pieces for p in (q['a'], q['b'])]
    foot = max((p for p in ends if p[1] > 0), key=lambda p: -p[2])
    chain, cur = [], foot
    while pieces:
        i, rev = next((i, False) for i, q in enumerate(pieces) if np.linalg.norm(q['a'] - cur) < 1e-6) if any(
            np.linalg.norm(q['a'] - cur) < 1e-6 for q in pieces) else next((i, True) for i, q in enumerate(pieces) if np.linalg.norm(q['b'] - cur) < 1e-6)
        q = pieces.pop(i)
        if rev: q = {**q, 'a': q['b'], 'b': q['a']}
        chain.append(q); cur = q['b']
    # Fillet the line-line corners; keep every tangent junction.
    out, corners = [], []
    for k, q in enumerate(chain):
        if q['arc']:
            mid_dir = (q['a'] - q['c']) + (q['b'] - q['c'])
            out.append({'kind': 'arc', 'start': q['a'].tolist(), 'end': q['b'].tolist(), 'center': q['c'].tolist(),
                        'mid': (q['c'] + q['r'] * mid_dir / np.linalg.norm(mid_dir)).tolist(), 'radiusM': q['r'],
                        'sourceIndex': q['idx']})
            continue
        a, b = q['a'].copy(), q['b'].copy()
        prev = chain[k - 1] if k else None; nxt = chain[k + 1] if k + 1 < len(chain) else None
        if prev is not None and not prev['arc']:
            a = a + (b - a) / np.linalg.norm(b - a) * corners[-1]['cut']   # trimmed by the fillet before it
        if nxt is not None and not nxt['arc']:
            u = (q['b'] - q['a']) / np.linalg.norm(q['b'] - q['a']); w = (nxt['b'] - nxt['a']) / np.linalg.norm(nxt['b'] - nxt['a'])
            theta = math.acos(np.clip(u @ w, -1, 1)); cut = R * math.tan(theta / 2)
            fa, fb = q['b'] - u * cut, q['b'] + w * cut
            n = w - (w @ u) * u; n /= np.linalg.norm(n); c = fa + n * R
            md = (fa - c) + (fb - c); mid = c + R * md / np.linalg.norm(md)
            corners.append({'cut': cut, 'angleDeg': math.degrees(theta), 'cornerM': q['b'].tolist(),
                            'maxDeviationM': R * (1 / math.cos(theta / 2) - 1)})
            b = fa
            out.append({'kind': 'line', 'start': a.tolist(), 'end': b.tolist(), 'lengthM': float(np.linalg.norm(b - a)), 'sourceIndex': q['idx']})
            out.append({'kind': 'arc', 'start': fa.tolist(), 'end': fb.tolist(), 'center': c.tolist(), 'mid': mid.tolist(),
                        'radiusM': R, 'angleDeg': math.degrees(theta), 'sourceIndex': None, 'addedFillet': True})
            continue
        out.append({'kind': 'line', 'start': a.tolist(), 'end': b.tolist(), 'lengthM': float(np.linalg.norm(b - a)), 'sourceIndex': q['idx']})
    for s, t in zip(out, out[1:]):
        assert np.linalg.norm(np.array(s['end']) - np.array(t['start'])) < 1e-9, 'path not continuous'
    assert len(corners) == 2, corners
    doc = {'schema': 'rfr.main-hoop-path/1', 'purpose': "team's Main Roll Hoop Sketch as one continuous weldment (F.5.6)",
           'sourceSketch': 'design/constraint-ledger.json hoops[0] (Main Roll Hoop Sketch)', 'profile': '1.000 x 0.095.SLDLFP',
           'addedFillets': corners, 'filletRadiusM': R,
           'note': 'All sketch lines and arcs kept; only the two 0.383 deg line-line corners get 60 in tangent arcs (about 10 mm long).',
           'segments': out}
    OUT.write_text(json.dumps(doc, indent=2) + '\n', encoding='utf-8')
    print(OUT.relative_to(ROOT), len(out), 'segments; fillets', [(round(c['angleDeg'], 3), f"{c['maxDeviationM'] * 1e6:.2f} um") for c in corners])


if __name__ == '__main__':
    main()
