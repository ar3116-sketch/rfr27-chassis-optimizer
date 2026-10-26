"""Write the bounded native chassis search specification and its cache inputs."""
from pathlib import Path
import json,sys
from frame_geometry import recipe
root=Path(__file__).resolve().parents[1]
inputs=['cad/working/candidate-005/RFR27-candidate-005.SLDPRT','cad/working/RFR27-working-starter.SLDPRT',
        'cad/working/percy/RFR27-Percy-seated-001.SLDASM','cad/working/candidate-005/RFR27-driver-layout-005.SLDASM',
        'design/constraint-ledger.json','design/percy-pose-001.json','scripts/evaluate-native-chassis.py','scripts/run-native.mjs',
        'scripts/native/bin/Debug/net8.0-windows/RfrEngineering.dll','sources/FSAE_Rules_2027_V1.pdf',
        'sources/2027 NMP TR Sim Procedure.docx','sources/VR3 Checklist.xlsx','sources/2027 New Member - TR Tracker.xlsx',
        'design/main-hoop-repair-002.json','handoff/evidence/native-beam-acceptance-final.json','scripts/validate-native-simulation.py',
        'cad/working/candidate-002/RFR27-candidate-002.SLDPRT','design/frame-005.json',
        'cad/working/candidate-005/RFR27-candidate-005.trim-progress.json',
        'scripts/frame_geometry.py','scripts/audit-frame-rules.py','scripts/make-driver-checks.py',
        'scripts/audit-manufacturing.py','scripts/validate-chassis-simulation.py',
        'handoff/evidence/engineering-2026-10-11T14-18-45-377Z/stdout.json']
inputs += [p.relative_to(root).as_posix() for p in sorted((root/'profiles').glob('*.SLDLFP'))]
inputs += [p.relative_to(root).as_posix() for p in sorted((root/'cad/working/percy/independent-v1').glob('*.SLDPRT')) if not p.name.startswith('~$')]
initial=recipe()['parameters']
bounds={'cockpitHalfWidthM':(.295,.335,.005,.001),'cockpitStationXM':(.91,1.0,.01,.002),
        'sideUpperZM':(.350,.390,.005,.001),'frontSideUpperZM':(.3455,.374,.002,.0005),
        'mainBraceZM':(1.246,1.285,.005,.001),'braceRearXM':(0,.10,.01,.002),
        'braceRearYM':(.26,.33,.005,.001),'braceRearZM':(.34,.42,.01,.002)}
variables=[{'name':n,'kind':'continuous','initial':initial[n],'lower':lo,'upper':hi,'initialStep':step,'minimumStep':minimum}
           for n,(lo,hi,step,minimum) in bounds.items()]
variables += [{'name':n,'kind':'categorical','initial':initial[n],'choices':choices} for n,choices in [('rearBayProfile',['B','C']),('floorDiagonal',[True,False])]]
spec={'schema':1,'scope':'New-member chassis: native weldments, physical trims, geometric rules and proposed driver packaging, prescribed native torsion',
      'variables':variables,'scoreAuthority':'native-solidworks-simulation','trackerTorqueLbFt':545,
      'evaluatorCommand':[sys.executable,'-X','utf8',str(root/'scripts/evaluate-native-chassis.py')],'timeoutSeconds':7200,
      'model':inputs[0],'evidenceInputs':inputs,
      'nativeSettings':{'baseline':inputs[1],'percy':inputs[2],'layout':inputs[3],'configuration':'Default<As Machined>',
                        'mainHoopRecipe':'design/main-hoop-repair-002.json','hoopsSource':'cad/working/candidate-002/RFR27-candidate-002.SLDPRT',
                        'reuseCandidateRecipe':'design/frame-005.json'}}
out=root/'design/optimizer-baseline.json';out.write_text(json.dumps(spec,indent=2)+'\n',encoding='utf-8');print(out)
