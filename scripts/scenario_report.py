"""Merge scenario_study output folders (e.g. cold and warm-started runs) into one table.

For every scenario id the best valid evaluation over all folders is kept (beam model, 75.5 lb
cap), with the cold/warm bests side by side, the node moves and the tube changes against the
reference design, and the reasons for infeasible scenarios.

  python scripts/scenario_report.py design/scenarios-009d design/scenarios-009d-warm --out design/scenarios-009d/combined
"""
from pathlib import Path
import argparse, json, re, sys

sys.path.insert(0, str(Path(__file__).parent))
from scenario_study import tube_changes

ROOT = Path(__file__).resolve().parents[1]
ORDER = lambda sid: (0 if sid.startswith('starter') else 1, sid)


def side(names):
    """Drop right-side twins whose left-side name is also listed (the frame is symmetric)."""
    to_left = lambda n: re.sub(r'(?<=[A-Za-z0-9])R(?=$|[_@\-])', 'L', n.split(':')[0])
    have = {n.split(':')[0] for n in names}
    return [n for n in names if to_left(n) == n.split(':')[0] or to_left(n) not in have]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dirs', nargs='+')
    ap.add_argument('--ref', default='design/frame-009c.json')
    ap.add_argument('--out', required=True, help='output stem (writes .json and .md)')
    a = ap.parse_args()
    ref = json.loads((ROOT / a.ref).read_text(encoding='utf-8-sig'))
    sc = {}
    for d in a.dirs:
        for f in sorted((ROOT / d).glob('*-search.json')):
            s = json.loads(f.read_text(encoding='utf-8')); sid = s['scenario']['id']
            e = sc.setdefault(sid, {'scenario': s['scenario'], 'runs': {}, 'reasons': set(), 'evaluations': 0})
            valid = [r for r in s['results'] if r['valid']]
            e['evaluations'] += len(s['results'])
            e['reasons'] |= {(r.get('reason') or '')[:220] for r in s['results'] if not r['valid']}
            if s['scenario'].get('note'): e['reasons'].add(s['scenario']['note'])
            if valid:
                b = max(valid, key=lambda r: r['ratio'])
                kind = 'warm' if 'warm' in d or b.get('start') == 'warm' else 'cold'
                e['runs'][kind] = {'dir': d, 'best': b, 'n': len(valid)}
    rows = []
    for sid in sorted(sc, key=ORDER):
        e = sc[sid]; s = e['scenario']
        row = {'id': sid, 'widthIn': s.get('accumulatorWidthIn'), 'axleFrac': s.get('axleFrac'), 'evaluations': e['evaluations'],
               'coldBest': e['runs'].get('cold', {}).get('best', {}).get('ratio'), 'warmBest': e['runs'].get('warm', {}).get('best', {}).get('ratio')}
        if not e['runs']:
            row.update(status='infeasible', reasons=sorted(e['reasons'] - {''})[:4]); rows.append(row); continue
        kind, run = max(e['runs'].items(), key=lambda kv: kv[1]['best']['ratio'])
        b = run['best']; rec = json.loads((ROOT / run['dir'] / f'{sid}.json').read_text(encoding='utf-8'))
        ch = tube_changes(rec, ref)
        row.update(status='valid', chosen=kind, recipe=f"{run['dir']}/{sid}.json", tr=b['tr'], weightLb=b['weightLb'], trPerLb=b['ratio'],
                   params={k: b['params'][k] for k in ('braceRearXM', 'braceRearYM', 'braceRearZM', 'batteryPostXM')},
                   halfshaftClearanceMm=b.get('halfshaftClearanceMm'), batteryClearanceMm=(b.get('battery') or {}).get('clearanceMm'),
                   apexT=rec.get('halfshaftCorridor', {}).get('rearFrameApexT'), box=rec.get('accumulatorBox', {}).get('placement'),
                   changesVs009c={**ch, 'addedLeft': side(ch['added']), 'removedLeft': side(ch['removed']), 'resizedLeft': side(ch['resized'])})
        rows.append(row)
    out = ROOT / a.out; out.parent.mkdir(parents=True, exist_ok=True)
    out.with_suffix('.json').write_text(json.dumps({'reference': a.ref, 'dirs': a.dirs, 'scenarios': rows}, indent=2, default=float) + '\n', encoding='utf-8')
    L = ['| Scenario | Width in | Axle % | TR lb-ft/deg | Weight lb | TR/lb | cold / warm best | BR y mm | BAT0 x mm | A-frame apex | Shaft clr mm | Box clr mm |',
         '|---|---|---|---|---|---|---|---|---|---|---|---|']
    for r in rows:
        w = r['widthIn'] or '18.0 (starter)'; ax = f"{r['axleFrac'] * 100:.0f}"
        if r['status'] != 'valid':
            L.append(f"| {r['id']} | {w} | {ax} | infeasible | | | | | | | | |"); continue
        cw = ' / '.join('-' if x is None else f'{x:.2f}' for x in (r['coldBest'], r['warmBest']))
        L.append(f"| {r['id']} | {w} | {ax} | {r['tr']:.0f} | {r['weightLb']:.2f} | {r['trPerLb']:.2f} | {cw} | {r['params']['braceRearYM'] * 1000:.0f} | "
                 f"{r['params']['batteryPostXM'] * 1000:.0f} | @{(r['apexT'] or 0.5) * 100:g} | {r['halfshaftClearanceMm']:.1f} | {r['batteryClearanceMm']:.1f} |")
    L.append('\nInfeasible:')
    for r in rows:
        if r['status'] != 'valid': L.append(f"- {r['id']}: " + '; '.join(r['reasons'][:2]))
    out.with_suffix('.md').write_text('\n'.join(L) + '\n', encoding='utf-8')
    print('\n'.join(L))


if __name__ == '__main__':
    main()
