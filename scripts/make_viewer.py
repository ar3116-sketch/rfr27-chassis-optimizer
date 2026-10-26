"""Build the interactive 3D frame comparison page (reports/frame-viewer.html).

Embeds each design's tubes (from its recipe), the supplied hoops, Percy's posed
segments and the keep-out solids, plus the metrics and where each number comes from.

  python scripts/make_viewer.py design/frame-005.json design/frame-007-free.json ...
"""
from pathlib import Path
import json, sys
import numpy as np
from scipy.spatial import ConvexHull

sys.path.insert(0, str(Path(__file__).parent))
import beam_model as bm
import frame_geometry as fg

ROOT = bm.ROOT
SW_005 = {'ratio': 9.79, 'tr': 684, 'weightLb': 69.87,
          'source': 'SolidWorks study of the shop-order rebuild (candidate-005v), validated (handoff/evidence/engineering-2026-10-12T01-12-38-471Z)'}


def hull_tris(points):
    p = np.asarray(points); h = ConvexHull(p)
    return {'v': np.round(p[h.vertices] * 1000, 1).tolist(),
            'f': [[int(np.where(h.vertices == i)[0][0]) for i in s] for s in h.simplices]}


def design_entry(path, base_groups):
    # 'design/frame-008a.json::75.5' labels a recipe whose file name has no cap in it.
    path, _, cap_label = path.partition('::')
    r = json.loads((ROOT / path).read_text(encoding='utf-8-sig'))
    meta = r.get('paramSearch') or r.get('layoutEngine', {}).get('verification') or {}
    short = None
    if path.endswith('frame-005.json'):
        name, metrics, short = "Codex's frame-005", SW_005, 'Codex'
    else:
        v = r.get('paramSearch') or r.get('layoutEngine', {}).get('verification', {})
        ratio = v.get('ratio', v.get('trPerLb')); tr = v.get('tr', v.get('trLbFtPerDeg'))
        metrics = {'ratio': round(ratio, 2), 'tr': round(tr), 'weightLb': round(v['weightLb'], 1),
                   'source': 'Fast beam model (matched SolidWorks within 0.4% TR / 0.3% weight on frame-005); not yet built and run in SolidWorks'}
        cap = (r.get('paramSearch', {}).get('params') and None) or r.get('layoutEngine', {}).get('maxWeightLb')
        stem = Path(path).stem
        cap = (cap_label if not cap_label.startswith('=') else None) or (stem.split('-cap')[1] if '-cap' in stem else None)
        name = f'Optimized, {cap} lb cap' if cap else 'Optimized, no weight cap'
        short = f'{cap} lb' if cap else 'No cap'
        if cap_label.startswith('='):   # explicit 'name|short' label
            name, _, short = cap_label[1:].partition('|'); short = short or name; cap = None
        if stem.startswith('frame-006'): name += ' (Codex nodes)'
        if stem.startswith('frame-007'): name += ' (round 1)'
        # A SolidWorks study of this exact recipe (pipeline `simulate`) replaces the model numbers.
        cid = stem.replace('frame-', '')
        # TR from the validated SolidWorks study of this recipe (the untrimmed copy '<id>u' when
        # the trimmed study left 0.750 in tubes unjoined), weight from the trimmed part's gates.
        score = next((ROOT / 'design' / f'score-{c}.json' for c in (cid, cid + 'u') if (ROOT / 'design' / f'score-{c}.json').exists()), None)
        gates = ROOT / 'design' / f'gates-{cid}.json'
        if score is not None:
            sw = json.loads(score.read_text(encoding='utf-8-sig'))
            w = sw.get('weightLb') or (json.loads(gates.read_text(encoding='utf-8-sig'))['chassis']['weightLb'] if gates.exists() else None)
            if sw.get('trLbFtPerDeg') and w:
                metrics = {'ratio': round(sw['trLbFtPerDeg'] / w, 2), 'tr': round(sw['trLbFtPerDeg']), 'weightLb': round(w, 2),
                           'source': f"SolidWorks: TR study {sw['evidence']}, weight of the trimmed weldment; beam model said {metrics['ratio']}",
                           'validated': True}
                name += ' - SolidWorks'
    tubes = []
    for g in r['groups']:
        kind = 'new' if g['name'] not in base_groups else 'core'
        for m in g['members']:
            s = bm.section(m['profile'])
            tubes.append({'n': m['name'], 'a': np.round(np.array(m['a']) * 1000, 1).tolist(), 'b': np.round(np.array(m['b']) * 1000, 1).tolist(),
                          'od': round(s['od'] * 1000, 2), 'p': Path(m['profile']).stem, 'k': kind})
    return {'name': name, 'short': short, 'file': path, 'metrics': metrics, 'members': len(r['members']),
            'params': r.get('paramSearch', {}).get('params'), 'tubes': tubes}


def main(paths):
    base = json.loads((ROOT / 'design/frame-005.json').read_text(encoding='utf-8-sig'))
    base_groups = {g['name'] for g in base['groups']}
    hoops = [np.round(np.array(p) * 1000, 1).tolist() for p in bm.hoops().values()]
    percy = json.loads((ROOT / 'design/percy-pose-001-points.json').read_text(encoding='utf-8-sig'))
    driver = []
    for part in percy['parts']:
        pts = np.array(part['pointsM'])
        if len(pts) >= 4:
            driver.append(hull_tris(pts + np.array([0, 0, 0])))
    _, refs = fg.source_geometry()
    keepouts = [{'name': b['name'], **hull_tris(b['verticesM'])} for b in refs]
    # Packaging zones estimated from photos of car 61 (scripts/photo_keepouts.py).
    import photo_keepouts
    keepouts += [{'name': z['name'], 'est': True, **hull_tris(z['verticesM'])} for z in photo_keepouts.load()]
    # Rear halfshaft corridors (estimated axle line), as recorded by the optimizer.
    hs = next((json.loads((ROOT / q.split('::')[0]).read_text(encoding='utf-8-sig')).get('halfshaftCorridor') for q in paths
               if json.loads((ROOT / q.split('::')[0]).read_text(encoding='utf-8-sig')).get('halfshaftCorridor')), None)
    if hs:
        ang = np.linspace(0, 2 * np.pi, 24, endpoint=False)
        ring = np.stack([hs['xM'] + hs['radiusM'] * np.cos(ang), np.zeros(24), hs['zM'] + hs['radiusM'] * np.sin(ang)], axis=1)
        for sgn in (1, -1):
            keepouts.append({'name': 'Halfshaft corridor', **hull_tris(np.vstack([ring + [0, sgn * hs['yInM'], 0], ring + [0, sgn * hs['yOutM'], 0]]))})
    # Every rule-valid design the node searches evaluated: [weight lb, ratio].
    cloud = []
    for log in sorted((ROOT / 'design').glob('frame-009-*-search.json')):   # round 3: final constraints
        for e in json.loads(log.read_text(encoding='utf-8-sig'))['results']:
            if e.get('valid') and e.get('ratio'): cloud.append([round(e['weightLb'], 2), round(e['ratio'], 3)])
    data = {'designs': [design_entry(p, base_groups) for p in paths], 'cloud': cloud, 'hoops': hoops, 'driver': driver, 'keepouts': keepouts,
            'pickups': [np.round(np.array(p) * 1000, 1).tolist() for p in base['loadPoints'] + base['fixturePoints']],
            'probes': [np.round(np.array(p) * 1000, 1).tolist() for p in base['probes']]}
    html = (ROOT / 'scripts/viewer_template.html').read_text(encoding='utf-8').replace('/*DATA*/null', json.dumps(data, separators=(',', ':')))
    out = ROOT / 'reports/frame-viewer.html'; out.write_text(html, encoding='utf-8')
    print(out, f'{len(html) / 1e6:.2f} MB', [d['name'] for d in data['designs']])


if __name__ == '__main__':
    main(sys.argv[1:])
