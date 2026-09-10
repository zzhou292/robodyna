"""Explicit V2 source assembly reuses whole geometry and attachment contracts."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from test_source_assembly import AssemblyFixture
from test_canonical_part_geometry import row


class AssemblyLaw44(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.f = AssemblyFixture(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def analytic_source(self, fail=None, curve=0):
        old = '\n'.join([row(38, '7.8900E-9', '2.0000E+5', .3, 180), row(8000, 8, 48, None, '0.0')])
        new = '\n'.join([row(38, '7.8900E-9', 1000, .3, 20, 10, fail), row(8000, 8, curve, None, '0.0')])
        self.assertIn(old.encode(), self.f.base_raw)
        self.f.rebuild(self.f.base_raw.replace(old.encode(), new.encode()))

    def test_opt_in_keeps_v1_table_geometry_and_interfaces_exact(self):
        old = self.f.compile_assembly()
        new = self.f.compile_assembly(material_policy='law44_tabulated_or_linear')
        self.assertEqual(old['schema'], 'robo-dyna.source-assembly-inventory.v1')
        self.assertEqual(new['schema'], 'robo-dyna.source-assembly-inventory.v2')
        for key in ('counts', 'geometry', 'attachments', 'boundary', 'parent_bindings'):
            self.assertEqual(old[key], new[key])
        for a, b in zip(old['declarations']['materials'], new['declarations']['materials']):
            self.assertEqual(b['hardening_model'], 'law44_tabulated')
            self.assertEqual(a, {k: v for k, v in b.items() if k != 'hardening_model'})

    def test_mixed_and_zero_curve_selections_preserve_explicit_null_association(self):
        self.analytic_source()
        with self.assertRaises(ValueError):
            self.f.compile_assembly()
        mixed = self.f.compile_assembly(material_policy='law44_tabulated_or_linear')
        self.assertEqual(mixed['counts']['curves'], 1)
        self.assertEqual([m['hardening_model'] for m in mixed['declarations']['materials']],
                         ['law44_tabulated', 'law44_linear'])
        for p in mixed['parent_bindings']:
            if p['source_part_id'] == 18:
                self.assertEqual(p['source_curve_id'], 0)
                self.assertIsNone(p['curve_index'])
            else:
                self.assertEqual(p['curve_index'], 0)
        single = self.f.compile_assembly(part_ids=(18,), material_policy='law44_tabulated_or_linear')
        self.assertEqual(single['counts']['curves'], 0)
        self.assertEqual(single['declarations']['curves'], [])
        self.assertFalse(single['simulation_ready'])
        self.assertFalse(single['full_attachment_closure_qualified'])
        self.assertEqual(single['geometry']['parts'][0], mixed['geometry']['parts'][1])

    def test_failure_missing_curve_and_unknown_policy_still_reject(self):
        for options in (dict(fail=.25), dict(curve=None), dict(curve=999)):
            self.analytic_source(**options)
            with self.assertRaises(ValueError):
                self.f.compile_assembly(material_policy='law44_tabulated_or_linear')
        self.analytic_source()
        with self.assertRaisesRegex(ValueError, 'material policy'):
            self.f.compile_assembly(material_policy='guess')
        self.assertEqual(self.f.compile_assembly(material_policy='law44_tabulated_or_linear')['counts']['shells'], 4)


if __name__ == '__main__':
    unittest.main()
