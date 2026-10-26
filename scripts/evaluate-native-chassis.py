"""Build, trim, check and solve one bounded native SolidWorks chassis candidate.
Every parameter is consumed. Failed or unavailable checks cannot score.
"""
from pathlib import Path
import hashlib,importlib.util,json,os,shutil,subprocess,sys,uuid
from frame_geometry import ROOT,recipe,load,save
NODE=Path(os.environ.get('RFR_NODE') or shutil.which('node') or 'C:/Program Files/nodejs/node.exe') # RFR_NODE, then PATH, then default
def module(name,file):
    s=importlib.util.spec_from_file_location(name,ROOT/'scripts'/file);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
rules=module('frame_rules','audit-frame-rules.py');driver=module('driver_checks','make-driver-checks.py')
manufacturing=module('manufacturing','audit-manufacturing.py');simulation=module('chassis_simulation','validate-chassis-simulation.py')
def digest(v):return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def owned(relative):
    p=(ROOT/relative).resolve()
    if not p.is_relative_to(ROOT/'cad'/'working'):raise ValueError('Input must be explicit working CAD')
    return str(p)
class NativeFailure(RuntimeError):
    def __init__(self,message,evidence):super().__init__(message);self.evidence=evidence
def native_job(folder,label,job):
    inp=folder/(label+'-job.json');save(inp,{**job,'timeoutMs':job.get('timeoutMs',600000)})
    run=subprocess.run([str(NODE),str(ROOT/'scripts/run-native.mjs'),str(inp)],cwd=ROOT,capture_output=True,text=True,encoding='utf-8',errors='replace',shell=False)
    (folder/(label+'-runner.txt')).write_text(run.stdout+'\n'+run.stderr,encoding='utf-8')
    records=[json.loads(s) for s in run.stdout.splitlines() if s.startswith('{')]
    evidence=next((Path(r['evidence']) for r in reversed(records) if 'evidence' in r),None)
    if not evidence or not (evidence/'stdout.json').exists():raise NativeFailure('Native runner produced no retained result',str(evidence))
    result=load(evidence/'stdout.json')
    if run.returncode!=0 or not result.get('ok'):raise NativeFailure(result.get('error','Native runner failed'),str(evidence))
    save(folder/(label+'.json'),result['result']);return result['result'],str(evidence)
def evaluate(request,folder):
    if Path(request['root']).resolve()!=ROOT:raise ValueError('Project root mismatch')
    settings=request['nativeSettings'];r=recipe(request['parameters']);rpath=folder/'frame-recipe.json';save(rpath,r)
    scalar=rules.audit(r);save(folder/'rule-scalars.json',scalar)
    layout=driver.layout(r);lpath=folder/'driver-layout.json';save(lpath,layout)
    gates=dict.fromkeys(request['requiredGates'],None);evidence={}
    result={'status':'blocked','requestHash':digest(request),'authority':'native-solidworks-simulation','gates':gates,'parameters':r['parameters'],'nativeEvidence':evidence,'metrics':{'trackerTorqueLbFt':545},'provenance':{'geometryRecipeSha256':digest(r)}}
    def finish():
        result['failedGates']=[k for k,v in gates.items() if v is False];result['unverifiedGates']=[k for k,v in gates.items() if v is None]
        result['status']='rejected' if result['failedGates'] else ('blocked' if result['unverifiedGates'] else 'complete');return result
    if not scalar['allImplementedChecksPass']:gates['rules']=False;result['error']='Frame parameters fail assignment rule geometry';return finish()
    if not layout['scalarPass']:gates['driverClearance']=False;result['error']='Proposed driver layout fails rule dimensions';return finish()
    restore=owned(settings['layout']);candidate=None
    if settings.get('reuseCandidateRecipe'):
        rr=load(ROOT/settings['reuseCandidateRecipe']);cp=Path(owned(request['model']));journal=cp.with_suffix('.trim-progress.json')
        if digest(rr)==digest(r) and journal.exists():
            j=load(journal)
            if j['nextTrimIndex']==len(r['trims']) and j['cadSha256']==sha(cp) and j['recipeSha256']==sha(ROOT/settings['reuseCandidateRecipe']):candidate=cp;evidence['reusedCheckpoint']=str(journal)
    if candidate is None:
        candidate=ROOT/'cad/working/optimizer'/('candidate-'+digest(request)[:10]+'-'+uuid.uuid4().hex[:6])/'chassis.SLDPRT'
        build_recipe=folder/'build-recipe.json';save(build_recipe,{**r,'trims':[]})
        _,evidence['build']=native_job(folder,'build',{'command':'build-connecting-frame','source':owned(settings['hoopsSource']),'output':str(candidate),'recipe':str(build_recipe),'restore':restore})
        for start in range(0,len(r['trims']),20):
            _,ev=native_job(folder,f'trims-{start:03}',{'command':'trim-connecting-frame','path':str(candidate),'recipe':str(rpath),'start':start,'count':min(20,len(r['trims'])-start),'restore':restore});evidence[f'trims{start}']=ev
    result['candidate']=str(candidate)
    snapshot=ROOT/'cad/tests'/('optimizer-audit-'+uuid.uuid4().hex[:12])/'chassis.SLDPRT'
    a,evidence['geometry']=native_job(folder,'geometry',{'command':'chassis-audit','path':str(candidate),'baseline':owned(settings['baseline']),'percy':owned(settings['percy']),'restore':restore,'configuration':settings['configuration'],'snapshot':str(snapshot),'mainHoopRecipe':str(ROOT/settings['mainHoopRecipe'])})
    for key in ('nativeRebuild','bodyValidity','protectedReferences','keepoutClearance','structuralOnlyMass'):gates[key]=a[key]
    gates['continuousMainHoop']=a['mainHoopAudit']['accepted'];result['metrics']['weightLb']=a['mass']['weightLb']
    result['provenance'].update({'cadSha256':a['sourceCadSha256'],'cadPath':str(candidate),'configuration':a['configuration'],'includedBodySetSha256':digest(sorted(b['name'] for b in a['bodies']))})
    if not a['tubeIntersectionsFree']:gates['allMembersTrimmed']=False
    if not a['driverSolidIntersectionFree']:gates['driverClearance']=False
    if any(v is False for v in gates.values()):result['error']='Native solid geometry rejected';return finish()
    fit,evidence['endFit']=native_job(folder,'end-fit',{'command':'joint-fit-audit','path':str(candidate),'recipe':str(rpath),'restore':restore,'timeoutMs':900000})
    gates['allMembersTrimmed']=fit['allStraightMemberEndsFit'] and a['tubeIntersectionsFree']
    packaging,evidence['packaging']=native_job(folder,'packaging',{'command':'packaging-audit','path':str(candidate),'recipe':str(rpath),'layoutRecipe':str(lpath),'restore':restore})
    reports={x['name']:x for x in packaging['reports']}
    gate_names=['RuleOpening','SuppliedOversizeOpening','RuleInternal','RuleInternalVerticalEntry','SuppliedOversizeInternal','SuppliedOversizeInternalVerticalEntry']
    gates['cockpitInsertion']=all(reports[n]['accepted'] for n in gate_names)
    gates['driverClearance']=layout['scalarPass'] and a['driverSolidIntersectionFree'] and all(reports[n]['accepted'] for n in ('RuleHip','RuleShoulder','RuleHelmet','RuleDriverOutline'))
    gates['rules']=scalar['allImplementedChecksPass'] and all(x['sectionValid'] for x in fit['reports']) and gates['allMembersTrimmed'] and gates['continuousMainHoop']
    production=manufacturing.audit(r,a,fit);save(folder/'manufacturing.json',production);gates['producibility']=production['geometryAccepted']
    if any(v is False for v in gates.values()):result['error']='Assignment geometry/fit/packaging gate rejected';return finish()
    solved=[]
    for name,mesh_scale,load_scale in [('default',1,1),('refined',.5,1),('reversed',1,-1)]:
        output=ROOT/'cad/tests'/('optimizer-study-'+uuid.uuid4().hex[:12])/('chassis-'+name+'.SLDPRT')
        s,evidence[name]=native_job(folder,'study-'+name,{'command':'chassis-simulation','source':str(candidate),'output':str(output),'recipe':str(rpath),'restore':restore,'meshScale':mesh_scale,'loadScale':load_scale,'timeoutMs':900000})
        checked=simulation.validate(s,r);save(folder/('study-'+name+'-validation.json'),checked);solved.append((s,checked))
    convergence=simulation.compare(*(x[1] for x in solved));save(folder/'mesh-and-reversal.json',convergence);s,checked=solved[0]
    for k in ('beamConnectivity','nativeSimulation','material','loadAndProbeMapping','reactionBalance','rotationSigns'):gates[k]=True
    result['metrics'].update({'rx1Deg':checked['rxDeg'][0],'rx2Deg':checked['rxDeg'][1],'actualTorqueLbFt':checked['actualTorqueLbFt']})
    result['provenance'].update({'materialSha256':digest({'databaseSha256':s['materialDatabaseSha256'],'beams':s['materials']}),'studySha256':digest(s),'solverVersion':s['solverVersion']})
    result['validation']={'default':checked,'meshAndReversal':convergence,'ruleScope':scalar['scope'],'manufacturingScope':production['scope']};return finish()
def main():
    inp,out=map(Path,sys.argv[1:3]);request=load(inp)
    try:result=evaluate(request,inp.parent);code=0
    except Exception as ex:
        result={'status':'blocked','requestHash':digest(request),'authority':'native-solidworks-simulation','error':str(ex),'nativeEvidence':getattr(ex,'evidence',None),'nativeStateUnknown':(ROOT/'handoff/native-cad.lock').exists()};code=1
    save(out,result);print(json.dumps({'status':result['status'],'response':str(out)}));return code
if __name__=='__main__':raise SystemExit(main())
