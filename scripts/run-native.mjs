// Run a typed native engineering job and retain exact inputs, outputs and exit status.
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {spawn} from 'node:child_process';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const jobPath=path.resolve(process.argv[2]??'');
const job=JSON.parse(fs.readFileSync(jobPath,'utf8').replace(/^\uFEFF/,''));
const timeoutMs=job.timeoutMs??180000;
if(!Number.isInteger(timeoutMs)||timeoutMs<1000||timeoutMs>900000)throw new Error('timeoutMs must be an integer between 1000 and 900000');
const evidence=path.join(root,'handoff/evidence/engineering-'+new Date().toISOString().replace(/[:.]/g,'-'));
fs.mkdirSync(evidence,{recursive:true});fs.writeFileSync(path.join(evidence,'input.json'),JSON.stringify(job,null,2));
const dll=path.join(root,'scripts/native/bin/Debug/net8.0-windows/RfrEngineering.dll');
// dotnet: RFR_DOTNET, then PATH, then the default install folder. Checked before the lock is taken.
const onPath=file=>(process.env.PATH??'').split(path.delimiter).filter(Boolean).map(d=>path.join(d,file)).find(p=>fs.existsSync(p));
const dotnet=process.env.RFR_DOTNET||onPath('dotnet.exe')||'C:/Program Files/dotnet/dotnet.exe';
if(!fs.existsSync(dotnet))throw new Error('dotnet not found at '+dotnet+'; set RFR_DOTNET to the full path of dotnet.exe');
const lockPath=path.join(root,'handoff/native-cad.lock');
const lock=fs.openSync(lockPath,'wx');
fs.writeFileSync(lock,JSON.stringify({pid:process.pid,jobPath,evidence,startedUtc:new Date().toISOString()},null,2));
const out=fs.openSync(path.join(evidence,'stdout.json'),'w');
const err=fs.openSync(path.join(evidence,'stderr.txt'),'w');
console.log(JSON.stringify({evidence,status:'running'}));
const child=spawn(dotnet,[dll,root,jobPath],{stdio:['ignore','pipe','pipe'],windowsHide:true});
let size=0,problem=null;
child.stdout.on('data',b=>{fs.writeSync(out,b);size+=b.length;if(size>32*1024*1024){problem='Output limit exceeded; CAD outcome unknown';child.kill();}});
child.stderr.on('data',b=>{fs.writeSync(err,b);process.stderr.write(b);});
const timer=setTimeout(()=>{problem='Native timeout; inspect CAD and lock evidence before manually releasing lock';child.kill();},timeoutMs);
const result=await new Promise(resolve=>{
  child.on('error',e=>{problem=e.message;resolve({exitCode:null,signal:null});});
  child.on('close',(exitCode,signal)=>resolve({exitCode,signal}));
});
clearTimeout(timer);fs.closeSync(out);fs.closeSync(err);fs.closeSync(lock);
if(!problem)fs.unlinkSync(lockPath);
fs.writeFileSync(path.join(evidence,'execution.json'),JSON.stringify({jobPath,...result,error:problem},null,2));
console.log(JSON.stringify({evidence,...result,error:problem}));
if(result.exitCode!==0||problem){console.error(fs.readFileSync(path.join(evidence,'stdout.json'),'utf8').slice(0,10000));process.exitCode=1;}
