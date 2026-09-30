"""V3 selects real elastic declarations without fabricating plastic inputs."""
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from test_source_assembly import AssemblyFixture
from test_canonical_part_geometry import row


class AssemblySections(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.f = AssemblyFixture(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def elastic(self, poisson=.3, optional=None):
        old = '\n'.join(['*MAT_PIECEWISE_LINEAR_PLASTICITY',
            row(38, '7.8900E-9', '2.0000E+5', .3, 180), row(8000, 8, 48, None, '0.0'), '', ''])
        new = '\n'.join(['*MAT_ELASTIC', row(38, '7.8900E-9', '2.0000E+5', poisson, optional)])
        self.assertIn(old.encode(), self.f.base_raw)
        self.f.rebuild(self.f.base_raw.replace(old.encode(), new.encode()))

    def compile(self, **kwargs):
        return self.f.compile_assembly(material_policy='layered_law1_or_law44', **kwargs)

    def test_explicit_law_tags_preserve_legacy_geometry_tables_and_boundary(self):
        old = self.f.compile_assembly(material_policy='law44_tabulated_or_linear')
        new = self.compile()
        self.assertEqual(new['schema'], 'robo-dyna.source-assembly-inventory.v3')
        for key in ('counts', 'geometry', 'boundary', 'parent_bindings'):
            self.assertEqual(old[key], new[key])
        self.assertEqual(old['attachments'], {k:v for k,v in new['attachments'].items() if k!='auxiliary_frontier'})
        self.assertEqual(new['attachments']['auxiliary_frontier']['source_node_ids'], [])
        for a, b in zip(old['declarations']['materials'], new['declarations']['materials']):
            self.assertEqual(b['material_law'], 'layered_law44')
            self.assertEqual(a, {k: v for k, v in b.items() if k != 'material_law'})

    def test_elastic_only_and_mixed_keep_optional_curve_and_original_source(self):
        self.elastic('-0.0')
        for policy in ('tabulated', 'law44_tabulated_or_linear'):
            with self.assertRaises(ValueError):
                self.f.compile_assembly(material_policy=policy)
        mixed = self.compile(boundary_policy='released_external_connections')
        pure = self.compile(part_ids=(18,), boundary_policy='released_external_connections')
        elastic = pure['declarations']['materials'][0]
        self.assertEqual(set(elastic), {'material_id', 'density_kg_m3', 'young_pa',
            'poisson_ratio', 'cards', 'source', 'material_law'})
        self.assertEqual(elastic['material_law'], 'layered_law1')
        self.assertEqual(struct.pack('d', elastic['poisson_ratio']), struct.pack('d', -0.0))
        self.assertEqual(elastic['young_pa'], 2e11)
        self.assertAlmostEqual(elastic['density_kg_m3'], 7890)
        self.assertEqual(elastic['source']['keyword'], '*MAT_ELASTIC')
        self.assertEqual(pure['counts']['curves'], 0)
        self.assertEqual(pure['declarations']['curves'], [])
        self.assertEqual(mixed['counts']['curves'], 1)
        self.assertEqual(pure['geometry']['parts'][0], mixed['geometry']['parts'][1])
        for p in mixed['parent_bindings']:
            if p['source_part_id'] == 18:
                self.assertEqual(p['source_curve_id'], 0)
                self.assertIsNone(p['curve_index'])
        self.assertFalse(pure['simulation_ready'])
        self.assertFalse(pure['full_attachment_closure_qualified'])
        self.assertFalse(pure['section_policy']['law1']['case_integration_qualified'])

    def test_invalid_elastic_options_and_missing_material_reject_then_retry(self):
        for poisson, optional in ((-.01, None), (.5, None), (.3, 0)):
            self.elastic(poisson, optional)
            with self.assertRaises(ValueError):
                self.compile()
        self.elastic()
        self.f.replace_source_only(self.f.raw.replace(row(18, 28, 38).encode(), row(18, 28, 999).encode()))
        with self.assertRaisesRegex(ValueError, 'unresolved material'):
            self.compile()
        self.elastic()
        self.assertEqual(self.compile()['counts']['shells'], 4)


if __name__ == '__main__':
    unittest.main()
