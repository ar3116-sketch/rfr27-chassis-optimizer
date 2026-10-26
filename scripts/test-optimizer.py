"""Unit tests of search/admission mechanics; synthetic fixture is NOT chassis evidence."""
import importlib.util,json,sys,tempfile,unittest
from pathlib import Path
p=Path(__file__).with_name('optimize-chassis.py');s=importlib.util.spec_from_file_location('optimizer',p);opt=importlib.util.module_from_spec(s);s.loader.exec_module(opt)
class OptimizerTests(unittest.TestCase):
 def request(self):return {'trackerTorqueLbFt':545}
 def valid(self):return {'requestHash':opt.digest(self.request()),'authority':'native-solidworks-simulation',
  'gates':dict.fromkeys(opt.REQUIRED_GATES,True),'provenance':dict.fromkeys(('cadSha256','configuration','materialSha256','studySha256','includedBodySetSha256','geometryRecipeSha256','solverVersion'),'UNIT_TEST_ONLY'),
  'metrics':{'weightLb':75.57,'rx1Deg':.277,'rx2Deg':.230,'trackerTorqueLbFt':545}}
 def test_tracker_arithmetic(self):self.assertAlmostEqual(opt.admitted(self.valid(),self.request())[0],28.449138291261235,places=10)
 def test_every_hard_gate_rejects_unknown_and_false(self):
  for g in opt.REQUIRED_GATES:
   for state in (None,False):
    r=self.valid();r['gates'][g]=state;self.assertIsNone(opt.admitted(r,self.request())[0])
 def test_cancellation_and_wrong_torque_rejected(self):
  for a,b,t in [(.2,-.2,545),(0,.2,545),(.2,.2,543.23)]:
   r=self.valid();r['metrics'].update(rx1Deg=a,rx2Deg=b,trackerTorqueLbFt=t);self.assertIsNone(opt.admitted(r,self.request())[0])
 def test_stale_evidence_rejected(self):
  r=self.valid();r['requestHash']='old';self.assertIsNone(opt.admitted(r,self.request())[0])
 def test_synthetic_search_and_cache(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);fixture=root/'synthetic.py';fixture.write_text('''import json,sys,hashlib
r=json.load(open(sys.argv[1]));x=r['parameters']['x'];t=.1+(x-.75)**2
out={'status':'complete','requestHash':hashlib.sha256(json.dumps(r,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest(),'authority':'native-solidworks-simulation','gates':dict.fromkeys(r['requiredGates'],True),'provenance':dict.fromkeys(['cadSha256','configuration','materialSha256','studySha256','includedBodySetSha256','geometryRecipeSha256','solverVersion'],'UNIT_TEST_ONLY'),'metrics':{'weightLb':50,'rx1Deg':t,'rx2Deg':t,'trackerTorqueLbFt':545}}
json.dump(out,open(sys.argv[2],'w'))
''')
   spec={'variables':[{'name':'x','kind':'continuous','lower':0,'upper':1,'initial':0,'initialStep':.25,'minimumStep':.125}],
    'scoreAuthority':'native-solidworks-simulation','evaluatorCommand':[sys.executable,str(fixture)],'evidenceInputs':['synthetic.py'],
    'trackerTorqueLbFt':545,'model':'SYNTHETIC_UNIT_TEST','nativeSettings':{},'timeoutSeconds':10}
   result=opt.search(spec,root,root/'run',20,27);self.assertEqual(result['incumbent']['parameters']['x'],.75)
   result2=opt.search(spec,root,root/'run',20,27);self.assertEqual(result2['nativeEvaluations'],0)
   fixture.write_text(fixture.read_text()+'\n# changes context\n')
   old=(root/'run/incumbent.json').read_bytes()
   with self.assertRaisesRegex(ValueError,'Inputs changed'):opt.search(spec,root,root/'run',20,27)
   self.assertEqual(old,(root/'run/incumbent.json').read_bytes())
   result3=opt.search(spec,root,root/'changed-run',20,27);self.assertGreater(result3['nativeEvaluations'],0)
 def failing_fixture(self,root,body,timeout=5):
  script=root/'backend.py';script.write_text(body)
  return {'variables':[{'name':'x','kind':'continuous','lower':0,'upper':1,'initial':0,'initialStep':.25,'minimumStep':.125}],
   'scoreAuthority':'native-solidworks-simulation','evaluatorCommand':[sys.executable,str(script)],'evidenceInputs':['backend.py'],
   'trackerTorqueLbFt':545,'model':'SYNTHETIC_UNIT_TEST','nativeSettings':{},'timeoutSeconds':timeout}
 def test_timeout_latches_recovery_across_restarts(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);spec=self.failing_fixture(root,'import time; print("started",flush=True);time.sleep(4)',.15)
   r=opt.search(spec,root,root/'run',5,27)
   self.assertEqual(r['stopReason'],'native_preflight_blocked');self.assertEqual(r['nativeEvaluations'],1)
   self.assertTrue((root/'run/RECOVERY_REQUIRED.json').exists())
   with self.assertRaisesRegex(RuntimeError,'Previous native outcome'):opt.search(spec,root,root/'run',5,27)
 def test_malformed_response_stops_and_retains_evidence(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);spec=self.failing_fixture(root,'import sys;open(sys.argv[2],"w").write("invalid json")')
   r=opt.search(spec,root,root/'run',5,27)
   self.assertEqual(r['nativeEvaluations'],1);self.assertIsNone(r['incumbent'])
   self.assertTrue((root/'run/RECOVERY_REQUIRED.json').exists())
   self.assertEqual(next((root/'run/evaluations').glob('*/response.json')).read_text(),'invalid json')
 def test_missing_executable_reports_block_without_recovery_latch(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);spec=self.failing_fixture(root,'')
   spec['evaluatorCommand']=[str(root/'nonexistent-executable')]
   r=opt.search(spec,root,root/'run',5,27)
   self.assertEqual(r['stopReason'],'native_preflight_blocked');self.assertIsNone(r['incumbent'])
   self.assertFalse((root/'run/RECOVERY_REQUIRED.json').exists())
 def test_concurrent_run_is_refused(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp)
   with opt.run_lock(root):
    with self.assertRaisesRegex(RuntimeError,'Another run owns'):
     with opt.run_lock(root):pass
   self.assertFalse((root/'optimizer.lock').exists())
if __name__=='__main__':unittest.main()
