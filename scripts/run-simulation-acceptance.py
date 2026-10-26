"""Run the real SolidWorks beam/API acceptance suite, serially, on fresh test copies."""
from pathlib import Path
import argparse, datetime, json, subprocess, sys, os, shutil
from contextlib import contextmanager

ROOT=Path(__file__).resolve().parents[1]
# RFR_NODE / RFR_DOTNET, then PATH, then the default install folders.
NODE=os.environ.get('RFR_NODE') or shutil.which('node') or 'C:/Program Files/nodejs/node.exe'
DOTNET=os.environ.get('RFR_DOTNET') or shutil.which('dotnet') or 'C:/Program Files/dotnet/dotnet.exe'

def save(p,v):p.write_text(json.dumps(v,indent=2)+'\n',encoding='utf-8')
@contextmanager
def session_lock():
    p=ROOT/'handoff/optimizer-session.lock'
    f=p.open('x',encoding='utf-8')
    try:
        json.dump({'pid':os.getpid(),'operation':'native-simulation-acceptance'},f);f.flush();yield
    finally:
        f.close();p.unlink()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run');args=ap.parse_args()
    folder=(ROOT/(args.run or ('cad/tests/native-acceptance-'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d-%H%M%S')))).resolve()
    if not folder.is_relative_to(ROOT/'cad/tests') or folder.exists():raise ValueError('Use a fresh directory under cad/tests')
    if (ROOT/'handoff/native-cad.lock').exists() or (ROOT/'handoff/optimizer-session.lock').exists():raise RuntimeError('Another CAD operation is active')
    with session_lock():run_suite(folder)
def run_suite(folder):
    folder.mkdir(parents=True)
    subprocess.run([DOTNET,'build','scripts/native/RfrEngineering.csproj','--no-restore','--nologo'],cwd=ROOT,check=True)
    restore=str(ROOT/'cad/working/candidate-002/RFR27-driver-layout-002.SLDASM')
    def run(name,job):
        jobfile=folder/(name+'-job.json');save(jobfile,job)
        print('Native acceptance: '+name,flush=True)
        completed=subprocess.run([NODE,str(ROOT/'scripts/run-native.mjs'),str(jobfile)],cwd=ROOT,capture_output=True,text=True,encoding='utf-8',errors='replace')
        (folder/(name+'-runner.txt')).write_text(completed.stdout+'\n'+completed.stderr,encoding='utf-8')
        records=[json.loads(line) for line in completed.stdout.splitlines() if line.startswith('{')]
        evidence=Path(records[-1]['evidence']) if records else None
        if completed.returncode or evidence is None:raise RuntimeError('Native operation failed; inspect '+str(folder/(name+'-runner.txt')))
        output=evidence/'stdout.json';result=json.loads(output.read_text(encoding='utf-8'))
        if not result.get('ok'):raise RuntimeError('Native failure: '+str(output))
        return output
    source=str(folder/'geometry/beam.SLDPRT')
    run('geometry',{'command':'simulation-beam-test','output':source,'restore':restore})
    beam=run('beam',{'command':'simulation-beam-solve','source':source,'output':str(folder/'beam/beam.SLDPRT'),'restore':restore})
    positive=run('frame',{'command':'frame-acceptance','output':str(folder/'frame/frame.SLDPRT'),'restore':restore,'loadScale':1})
    negative=run('reverse',{'command':'frame-acceptance','output':str(folder/'reverse/frame.SLDPRT'),'restore':restore,'loadScale':-1})
    for evidence in (beam,positive,negative):
        a=json.loads(evidence.read_text(encoding='utf-8'))['result']
        if a['softSprings'] or a['inertialRelief']:raise RuntimeError('Artificial stabilization enabled')
    summary=folder/'acceptance.json'
    subprocess.run([sys.executable,'-X','utf8',str(ROOT/'scripts/validate-native-simulation.py'),'--beam',str(beam),'--frame',str(positive),'--reverse',str(negative),'--output',str(summary)],cwd=ROOT,check=True)
    print('Verified native acceptance: '+str(summary),flush=True)
if __name__=='__main__':main()
