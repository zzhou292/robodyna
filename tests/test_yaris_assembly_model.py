"""Explicit original-asset inventory gate; missing declared assets fail."""
import json
import os
from pathlib import Path
import sys
import unittest

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
from modelio.yaris_assembly import compile_archive_assembly, YARIS_CONNECTOR_PARTS
from modelio.yaris_part import compile_archive_part


class OriginalYarisAssembly(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        value = os.environ.get('YARIS_VEHICLE_ASSET_DIR')
        if not value:
            raise ValueError('YARIS_VEHICLE_ASSET_DIR must name the declared original asset')
        cls.assets = Path(value)
        cls.archive = cls.assets / 'source_model.zip'
        cls.single_before = json.dumps(compile_archive_part(cls.archive), sort_keys=True).encode()
        cls.report = compile_archive_assembly(cls.archive, cls.assets,
                                              boundary_policy='released_external_connections')

    def test_exact_whole_part_and_material_coverage(self):
        r = self.report
        self.assertEqual(r['selected_part_ids'], list(YARIS_CONNECTOR_PARTS))
        self.assertEqual(r['counts'], dict(parts=6, shells=915, nodes=1030, q4=804, native_t3=111,
                                          shared_nodes=0, materials=6, sections=6, curves=2,
                                          internal_nodal_rigid_groups=6, outgoing_nodal_rigid_groups=4,
                                          internal_spotwelds=0, outgoing_spotwelds=13, external_nodes=37))
        self.assertEqual(r['source']['member_sha256'],
                         '67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301')
        parts = r['geometry']['parts']
        self.assertEqual([len(p['shells']) for p in parts], [73, 138, 548, 94, 33, 29])
        self.assertEqual([len(p['nodes']) for p in parts], [98, 160, 584, 117, 36, 35])
        sections = r['declarations']['sections']
        self.assertEqual([s['source_elform'] for s in sections], [16, 2, 16, 2, 16, 16])
        self.assertEqual([s['thickness_m'][0] for s in sections],
                         [t * .001 for t in (.731, 3.845, .889, 1.648, 2.35, 2.35)])
        curves = r['declarations']['curves']
        self.assertEqual([(c['curve_id'], len(c['stress_pa']), c['stress_pa'][0], c['stress_pa'][-1])
                          for c in curves], [(2100180, 17, 180e6, 410e6), (2100270, 46, 270e6, 362e6)])
        parents = r['parent_bindings']
        self.assertEqual(len(parents), 915)
        self.assertEqual((parents[-1]['source_element_id'], parents[-1]['source_part_id'],
                          parents[-1]['source_material_id']), (2235319, 2000260, 2000260))
        radiator = [p for p in parents if p['source_part_id'] == 2000145]
        self.assertEqual(len(radiator), 548)
        self.assertEqual({(p['material_index'], p['section_index'], p['curve_index']) for p in radiator}, {(2, 2, 0)})

    def test_all_internal_groups_outgoing_groups_welds_and_tied_scope(self):
        a = self.report['attachments']
        self.assertEqual([g['rigid']['identity'] for g in a['nodal_rigid_groups'] if g['classification'] == 'internal'],
                         list(range(2200909, 2200915)))
        self.assertEqual([g['rigid']['identity'] for g in a['nodal_rigid_groups'] if g['classification'] == 'outgoing'],
                         [2200054, 2200055, 2200921, 2200946])
        self.assertEqual([w['weld']['identity'] for w in a['spotwelds']],
                         [2101297, 2101479, 2101480, 2101481] + list(range(2101483, 2101492)))
        self.assertEqual(a['whole_source_spotweld_record_count'], 2828)
        self.assertEqual([p['part_id'] for p in a['external_parts']],
                         [2000078, 2000121, 2000134, 2000139, 2000142, 2000204])
        self.assertEqual({n['source_id'] for n in a['frontier_incidence']['nodes']}, set(a['external_node_ids']))
        self.assertTrue(a['frontier_incidence']['source_records_verified'])
        self.assertEqual(a['tied_candidates'][0]['selected_master_part_ids'],
                         [2000119, 2000145, 2000157, 2000165, 2000260])
        self.assertFalse(a['tied_candidates'][0]['actual_pairing_qualified'])
        self.assertEqual(a['tied_candidates'][0]['candidate_pair_scope'], 'cross_boundary_only')
        released = self.report['boundary']['released_external_tied_source_scopes']
        self.assertEqual(len(released), 1)
        self.assertEqual(released[0]['source_identity'], ('yaris-coarse-v1l.key', 36))
        self.assertEqual(released[0]['selected_slave_part_ids'], [])
        self.assertEqual(len(released[0]['selected_master_part_ids']), 5)
        self.assertFalse(released[0]['actual_pairing_qualified'])
        self.assertFalse(self.report['simulation_ready'])
        self.assertFalse(self.report['full_attachment_closure_qualified'])
        self.assertFalse(self.report['boundary']['applied_to_dynamics'])

    def test_single_part_compiler_remains_byte_identical_after_assembly(self):
        after = json.dumps(compile_archive_part(self.archive), sort_keys=True).encode()
        self.assertEqual(self.single_before, after)


if __name__ == '__main__':
    unittest.main()
