"""Serial, resumable constrained search around a native chassis evaluator.

The evaluator command takes request.json and response.json arguments. It must build,
trim, check and solve a candidate and return the evidence contract below. Unknown or
failed hard gates cannot become an incumbent. This module never substitutes a beam
surrogate for the native score. Run --help for the command line.
"""
from __future__ import annotations
import argparse, hashlib, json, math, os, random, subprocess, time
from contextlib import contextmanager
from pathlib import Path

VERSION='rfr-native-pattern-search-3'
REQUIRED_GATES=('nativeRebuild','bodyValidity','continuousMainHoop','allMembersTrimmed','beamConnectivity',
 'rules','protectedReferences','driverClearance','cockpitInsertion','keepoutClearance',
 'producibility','nativeSimulation','structuralOnlyMass','material','loadAndProbeMapping',
 'reactionBalance','rotationSigns')
def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix(path.suffix+'.tmp');temp.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n');temp.replace(path)
def read(path): return json.loads(path.read_text(encoding='utf-8-sig'))
def file_digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def context(spec,root):
    paths=spec['evidenceInputs']
    if not paths: raise ValueError('Cache context needs geometry, profiles, rules, procedure, material and adapter inputs')
    return {'version':VERSION,'spec':spec,'files':{p:file_digest((root/p).resolve()) for p in paths}}
def validate_spec(spec):
    names=[v['name'] for v in spec['variables']]
    if len(names)!=len(set(names)): raise ValueError('Duplicate variable name')
    for v in spec['variables']:
        if v['kind']=='continuous':
            if not all(math.isfinite(v[k]) for k in ('lower','initial','upper','initialStep','minimumStep')): raise ValueError('Nonfinite bound')
            if not v['lower']<=v['initial']<=v['upper'] or not 0<v['minimumStep']<=v['initialStep']: raise ValueError('Invalid search bounds')
        elif v['kind']=='categorical':
            if v['initial'] not in v['choices'] or len(v['choices'])!=len(set(v['choices'])): raise ValueError('Invalid stock choices')
        else: raise ValueError('Unsupported variable kind')
    if not spec['evaluatorCommand'] or not all(isinstance(x,str) for x in spec['evaluatorCommand']): raise ValueError('Evaluator must be explicit argv')
    if spec.get('scoreAuthority')!='native-solidworks-simulation': raise ValueError('Only native Simulation can admit an incumbent')
    if spec.get('trackerTorqueLbFt')!=545: raise ValueError('The tracker convention is 545 lb-ft')
    if not math.isfinite(spec.get('timeoutSeconds',600)) or spec.get('timeoutSeconds',600)<=0: raise ValueError('Invalid evaluator timeout')

@contextmanager
def run_lock(run,name='optimizer.lock'):
    run.mkdir(parents=True,exist_ok=True)
    path=run/name
    try: fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL)
    except FileExistsError: raise RuntimeError(f'Another run owns {path}; inspect its PID and native CAD state before recovery') from None
    try:
        with os.fdopen(fd,'w') as f: json.dump({'pid':os.getpid(),'startedUtcUnix':time.time()},f)
        yield
    finally: path.unlink()
def admitted(result,request):
    reasons=[]
    if result.get('requestHash')!=digest(request): reasons.append('stale_or_mismatched_request')
    if result.get('authority')!='native-solidworks-simulation': reasons.append('wrong_score_authority')
    gates=result.get('gates',{})
    reasons += ['gate:'+g for g in REQUIRED_GATES if gates.get(g) is not True]
    provenance=result.get('provenance',{})
    for k in ('cadSha256','configuration','materialSha256','studySha256','includedBodySetSha256','geometryRecipeSha256','solverVersion'):
        if not provenance.get(k): reasons.append('missing_provenance:'+k)
    score=None
    try:
        m=result['metrics'];w=float(m['weightLb']);a=float(m['rx1Deg']);b=float(m['rx2Deg']);torque=float(m['trackerTorqueLbFt'])
        if not all(math.isfinite(x) for x in (w,a,b,torque)) or w<=0 or a*b<=0 or torque!=request['trackerTorqueLbFt']:
            reasons.append('invalid_weight_torque_or_signed_rotations')
        else:
            theta=abs((a+b)/2);score=torque/(w*theta)
            if not math.isfinite(score):reasons.append('nonfinite_score')
    except (KeyError,ValueError,TypeError,ZeroDivisionError):reasons.append('missing_or_invalid_metrics')
    return (None if reasons else score),reasons
def proposals(x,variables,steps):
    for v in variables:
        n=v['name']
        choices=v['choices'] if v['kind']=='categorical' else [max(v['lower'],x[n]-steps[n]),min(v['upper'],x[n]+steps[n])]
        for value in choices:
            if value!=x[n]: yield {**x,n:value}

def search(spec,root,run,budget,seed):
    with run_lock(root/'handoff','optimizer-session.lock'),run_lock(run):
        return search_locked(spec,root,run,budget,seed)

def search_locked(spec,root,run,budget,seed):
    validate_spec(spec);ctx=context(spec,root);ctx_hash=digest(ctx)
    if budget<1:raise ValueError('Budget must be positive')
    if (run/'RECOVERY_REQUIRED.json').exists():raise RuntimeError('Previous native outcome is unknown. Inspect RECOVERY_REQUIRED.json and CAD before starting another run.')
    previous=run/'context.json'
    if previous.exists() and digest(read(previous))!=ctx_hash:
        raise ValueError('Inputs changed: use a new --run directory. Existing evidence and incumbent were preserved.')
    save(previous,ctx)
    rng=random.Random(seed);variables=spec['variables'];initial={v['name']:v['initial'] for v in variables}
    incumbent=None;calls=0;attempts=0;history=[];stop_reason='evaluation_budget'
    started=time.time();cache=run/'cache';cache.mkdir(parents=True,exist_ok=True)
    def evaluate(x):
        nonlocal calls,attempts,incumbent
        request={'schema':1,'contextHash':ctx_hash,'parameters':x,'trackerTorqueLbFt':spec['trackerTorqueLbFt'],
                 'root':str(root),'model':spec['model'],'requiredGates':list(REQUIRED_GATES),'nativeSettings':spec['nativeSettings']}
        key=digest(request);cached=cache/(key+'.json');attempts+=1
        if cached.exists():result=read(cached);was_cached=True
        else:
            if calls>=budget:return None
            calls+=1;ordinal=len(list((run/'evaluations').glob('*')))+1
            folder=run/'evaluations'/f'{ordinal:05d}-{key[:12]}';folder.mkdir(parents=True,exist_ok=False)
            inp=folder/'request.json';out=folder/'response.json';save(inp,request)
            print(json.dumps({'event':'native_evaluation_started','parameters':x,'evidence':str(folder)}),flush=True)
            t=time.time()
            try:
                p=subprocess.run(spec['evaluatorCommand']+[str(inp),str(out)],cwd=root,shell=False,
                  capture_output=True,text=True,timeout=spec.get('timeoutSeconds',600),encoding='utf-8',errors='replace')
                (folder/'stdout.txt').write_text(p.stdout,encoding='utf-8');(folder/'stderr.txt').write_text(p.stderr,encoding='utf-8')
                result=read(out) if out.exists() else {'status':'blocked','error':'Evaluator returned no response','exitCode':p.returncode,'nativeStateUnknown':True}
                if not isinstance(result,dict):raise ValueError('Evaluator response must be a JSON object')
                if p.returncode!=0:result={**result,'status':'blocked','processExitCode':p.returncode}
            except subprocess.TimeoutExpired as ex:
                # Do not issue another CAD mutation after an unknown timeout outcome.
                for stream in ('stdout','stderr'):
                    value=getattr(ex,stream,None) or b''
                    (folder/(stream+'.txt')).write_text(value.decode('utf-8',errors='replace') if isinstance(value,bytes) else value,encoding='utf-8')
                result={'status':'blocked','error':'Native evaluator timed out; verify CAD state before resuming','nativeStateUnknown':True}
            except (ValueError,OSError) as ex:
                result={'status':'blocked','error':str(ex),'nativeStateUnknown':not isinstance(ex,OSError)}
            result['elapsedSeconds']=time.time()-t
            if result.get('nativeStateUnknown'):save(run/'RECOVERY_REQUIRED.json',{'requestHash':key,'evaluation':str(folder),'result':result})
            if result.get('status') in ('complete','rejected'):save(cached,result)
            was_cached=False
        score,reasons=admitted(result,request)
        if result.get('status')!='complete':score=None;reasons+=['evaluation_not_complete']
        entry={'parameters':x,'requestHash':key,'cached':was_cached,'status':result.get('status'),'score':score,'rejectionReasons':reasons,'error':result.get('error')}
        history.append(entry)
        if score is not None and (incumbent is None or score>incumbent['score']):
            incumbent={**entry,'evidence':result};save(run/'incumbent.json',incumbent)
        save(run/'progress.json',{'calls':calls,'attempts':attempts,'incumbent':incumbent,'last':entry})
        print(json.dumps(entry),flush=True)
        return entry
    base=evaluate(initial)
    if base['status'] in ('blocked','failed'):stop_reason='native_preflight_blocked'
    elif base['score'] is None:
        stop_reason='baseline_infeasible_or_unverified' # no blind search around an unvalidated generator
    else:
        steps={v['name']:v['initialStep'] for v in variables if v['kind']=='continuous'}
        x=initial
        while calls<budget:
            old_best=incumbent['score'];moved=False
            trial_list=list(proposals(x,variables,steps));rng.shuffle(trial_list)
            for trial in trial_list:
                e=evaluate(trial)
                if e is None:break
                if e['status'] in ('blocked','failed'):stop_reason='native_evaluator_blocked';break
                if incumbent['score']>old_best:x=incumbent['parameters'];moved=True;break
            if stop_reason=='native_evaluator_blocked':break
            if calls>=budget:break
            if not moved:
                if not steps or all(steps[v['name']]<=v['minimumStep'] for v in variables if v['kind']=='continuous'):
                    stop_reason='bounded_poll_exhausted';break
                steps={v['name']:max(v['minimumStep'],steps[v['name']]/2) for v in variables if v['kind']=='continuous'}
    summary={'schema':1,'status':'blocked' if 'blocked' in stop_reason else 'finished','stopReason':stop_reason,
       'nativeEvaluations':calls,'attempts':attempts,'seed':seed,'elapsedSeconds':time.time()-started,
       'contextHash':ctx_hash,'incumbent':incumbent,'history':history,'globalOptimalityClaim':False}
    save(run/'summary.json',summary);return summary
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--spec',type=Path,required=True)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--budget',type=int,default=30);p.add_argument('--seed',type=int,default=2701)
    a=p.parse_args();spec=read(a.spec);root=Path(__file__).resolve().parents[1]
    result=search(spec,root,a.run.resolve(),a.budget,a.seed)
    return 2 if result['incumbent'] is None else 0
if __name__=='__main__':raise SystemExit(main())
