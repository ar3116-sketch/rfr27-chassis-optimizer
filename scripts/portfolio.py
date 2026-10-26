"""Portfolio of warm starts at one frame geometry (WORK_LOG 2026-10-20, recreated from the 20 Oct
scratchpad script). The inner layout search (param_search.evaluate) runs once per reference
design, each time starting from that design's tubes (param_search.warm_state), and the best valid
result is kept. Different starts reach different local optima; frame-011b came from this.

  python scripts/portfolio.py --params design/frame-011b.json --refs design/frame-*.json \
      --skeleton free --workers 12 --out design/frame-013-portfolio.json

--refs takes recipe paths and globs, or a .txt/.json file listing them. Results: --out (best
recipe, with paramSearch provenance) and <out>-summary.json (one row per reference).
"""
import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'): os.environ.setdefault(_v, '1')
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import argparse, glob, json, sys, time, types

sys.path.insert(0, str(Path(__file__).parent))
import param_search as ps

ROOT = ps.ROOT


def load(p):
    return json.loads(Path(p).read_text(encoding='utf-8-sig'))


def params_of(path):
    r = load(path)
    return r.get('paramSearch', {}).get('params') or r['parameters']


def expand(items):
    out = []
    for it in items:
        p = Path(it) if Path(it).is_absolute() else ROOT / it
        if p.suffix == '.txt' and p.exists():
            out += expand([l.strip() for l in p.read_text(encoding='utf-8-sig').splitlines() if l.strip()]); continue
        if p.suffix == '.json' and p.exists():
            body = json.loads(p.read_text(encoding='utf-8-sig'))
            if isinstance(body, list): out += expand(body); continue   # a list of references
        hits = sorted(glob.glob(str(p))) or ([str(p)] if p.exists() else [])
        out += [h for h in hits if not h.endswith('-search.json')]
    seen = set()
    return [x for x in out if not (x in seen or seen.add(x))]


def run(ref, params, opt):
    args = types.SimpleNamespace(catalog='vr3', restarts=opt['restarts'], seed=opt['seed'], budget=opt['budget'],
                                 max_weight=opt['max_weight'], main_hoop='source', limits=None, packaging=None,
                                 warm=load(ref), skeleton=opt['skeleton'])
    t = time.time()
    try:
        r = ps.evaluate(params, args, budget=opt['budget'])
    except Exception as e:
        return {'ref': ref, 'valid': False, 'reason': repr(e)[:300]}
    return {'ref': ref, **{k: r.get(k) for k in ('valid', 'reason', 'ratio', 'tr', 'weightLb', 'trWork', 'members', 'skeleton')},
            'seconds': round(time.time() - t), 'recipe': r.get('recipe')}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--params', required=True, help='recipe whose geometry parameters are used (paramSearch.params or parameters)')
    ap.add_argument('--refs', nargs='+', required=True)
    ap.add_argument('--skeleton', choices=['fixed', 'free'], default='fixed')
    ap.add_argument('--max-weight', type=float, default=75.5)
    ap.add_argument('--budget', type=float, default=240)
    ap.add_argument('--restarts', type=int, default=2)
    ap.add_argument('--seed', type=int, default=51)
    ap.add_argument('--workers', type=int, default=12)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    params = params_of(ROOT / a.params); refs = expand(a.refs)
    opt = {'skeleton': a.skeleton, 'max_weight': a.max_weight, 'budget': a.budget, 'restarts': a.restarts, 'seed': a.seed}
    print(f'{len(refs)} references, skeleton {a.skeleton}, params from {a.params}', flush=True)
    res = []
    with ProcessPoolExecutor(max_workers=a.workers, initializer=ps._worker_init) as pool:
        futs = [pool.submit(run, r, params, opt) for r in refs]
        for f in as_completed(futs):
            r = f.result(); res.append(r)
            print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items() if k != 'recipe'}), flush=True)
    ok = sorted((r for r in res if r.get('valid')), key=lambda r: -r['ratio'])
    out = ROOT / a.out
    if ok:
        best = ok[0]
        prov = {'params': params, **{k: best[k] for k in ('valid', 'ratio', 'tr', 'weightLb', 'trWork', 'skeleton')},
                'start': 'warm', 'seed': a.seed, 'tag': f"portfolio ({a.skeleton} skeleton) at {a.params} params, warm from {best['ref']}"}
        out.write_text(json.dumps({**best['recipe'], 'paramSearch': prov}, indent=2) + '\n', encoding='utf-8')
    summary = out.with_name(out.stem + '-summary.json')
    summary.write_text(json.dumps({'args': vars(a), 'results': sorted([{k: v for k, v in r.items() if k != 'recipe'} for r in res],
                                                                    key=lambda r: -(r.get('ratio') or 0) if r.get('valid') else 0)},
                                  indent=1) + '\n', encoding='utf-8')
    print('BEST', json.dumps({k: v for k, v in ok[0].items() if k != 'recipe'}) if ok else 'none valid', flush=True)


if __name__ == '__main__':
    main()
