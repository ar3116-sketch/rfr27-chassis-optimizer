"""Rewrite an optimizer recipe with SolidWorks-safe member/group names (see
ground_structure.cad_name) and regenerate its cope list. For recipes written before
to_recipe produced safe names.

  python scripts/cad_names.py design/frame-008a.json
"""
from pathlib import Path
import json, re, sys
sys.path.insert(0, str(Path(__file__).parent))
import ground_structure as gs
from frame_geometry import make_trims


def convert(rec):
    xb, rename = {}, {}
    for g in rec['groups']:
        for m in g['members']:
            new = gs.cad_name(m['name'], xb)
            if new != m['name']: m.setdefault('optimizerName', m['name']); rename[m['name']] = new; m['name'] = new
        if len(g['members']) == 1 and g['name'].startswith(('Weld_Cand_', 'Weld_XT_', 'Weld_XA_', 'Weld_XB_')):
            g['name'] = 'Weld_' + g['members'][0]['name']
        g['name'] = re.sub(r'[^A-Za-z0-9_-]', '-', g['name'])
    by_name = {m['name']: m for g in rec['groups'] for m in g['members']}
    rec['members'] = [by_name[rename.get(m['name'], m['name'])] for m in rec['members']]
    hoop_nodes = {'MainHoop_1': [n for n in rec['nodes'] if n.startswith('MH')], 'FrontHoop_1': [n for n in rec['nodes'] if n.startswith('FH')]}
    rec['trims'] = make_trims(rec['groups'], rec['nodes'], hoop_nodes)
    return rec, rename


if __name__ == '__main__':
    for path in sys.argv[1:]:
        p = gs.ROOT / path; rec, rename = convert(json.loads(p.read_text(encoding='utf-8-sig')))
        bad = [n for g in rec['groups'] for n in [g['name']] + [m['name'] for m in g['members']] if re.search(r'[^A-Za-z0-9_-]', n)]
        if bad: sys.exit(f'Unsafe names remain: {bad[:5]}')
        p.write_text(json.dumps(rec, indent=2) + '\n', encoding='utf-8')
        print(f"{path}: {len(rename)} members renamed, {len(rec['groups'])} groups, {len(rec['trims'])} trims")
