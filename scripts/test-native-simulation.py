"""Regression tests against retained native acceptance and an actual disconnected failure."""
import copy,importlib.util,json,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=Path(__file__).with_name('validate-native-simulation.py');s=importlib.util.spec_from_file_location('native_validation',p);v=importlib.util.module_from_spec(s);s.loader.exec_module(v)
def data(stamp):return json.loads((ROOT/'handoff/evidence'/stamp/'stdout.json').read_text(encoding='utf-8'))['result']
class NativeValidationTests(unittest.TestCase):
 def setUp(self):self.a=data('engineering-2026-10-11T16-47-20-424Z')
 def test_real_connected_frame_passes(self):self.assertTrue(v.frame_test(self.a)['passed'])
 def test_real_disconnected_solve_is_rejected_despite_zero_solver_code(self):
  a=data('engineering-2026-10-11T16-45-36-723Z');self.assertEqual(a['solve'],0)
  with self.assertRaisesRegex(ValueError,'Disconnected'):v.frame_test(a)
 def test_wrong_loaded_node_is_rejected(self):
  self.a['loads'][0]['pointM'][0]=.8
  with self.assertRaises(ValueError):v.frame_test(self.a)
 def test_wrong_force_is_rejected(self):
  self.a['loads'][0]['forceN'][2]*=2
  with self.assertRaisesRegex(ValueError,'magnitudes'):v.frame_test(self.a)
 def test_unbalanced_native_reaction_is_rejected(self):
  self.a['reaction'][1]+=10
  with self.assertRaisesRegex(ValueError,'reaction balance'):v.frame_test(self.a)
 def test_wrong_rotation_sign_is_rejected(self):
  self.a['rotation'][1]*=-1
  with self.assertRaisesRegex(ValueError,'rotation signs'):v.frame_test(self.a)
 def test_accidental_rotation_fixture_is_rejected(self):
  with tempfile.TemporaryDirectory() as tmp:
   item=next(f for f in self.a['export']['files'] if f['path'].lower().endswith('.dat'))
   source=Path(item['path']).read_text();changed=source.replace(',123,0.00000,',',123456,0.00000,',1)
   self.assertNotEqual(changed,source);out=Path(tmp)/'audit.DAT';out.write_text(changed)
   item.update(path=str(out),sha256=v.sha(out))
   with self.assertRaisesRegex(ValueError,'translation-only'):v.frame_test(self.a)
if __name__=='__main__':unittest.main()
