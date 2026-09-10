"""Independent whole-part, mixed-material and complete-frontier host gates."""
from dataclasses import replace
from io import BytesIO
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
from modelio import yaris_part, yaris_assembly
from modelio._legacy import file_sha256, sha256
from modelio.assembly_declarations import AssemblyLimits
from modelio.spotweld_cards import scan_spotwelds
from modelio.yaris_assembly import compile_archive_assembly, write_assembly_report
from test_source_attachments import AttachedCanonical, README
from test_canonical_part_geometry import row


class AssemblyFixture(AttachedCanonical):
    def __init__(self, directory):
        super().__init__(directory)
        raw = self.raw.replace(('neighbor 18\n' + row(18, 27, 37)).encode(),
                               ('neighbor 18\n' + row(18, 28, 38)).encode())
        additions = ['*SECTION_SHELL', row(28, 16, None, 3), row(2., 2., 2., 2.),
                     '*MAT_PIECEWISE_LINEAR_PLASTICITY', row(38, '7.8900E-9', '2.0000E+5', .3, 180),
                     row(8000, 8, 48, None, '0.0'), '', '', '*DEFINE_CURVE', row(48, 0, 1, 1),
                     row(0, 180, width=20), row(.2, 230, width=20), row(.5, 410, width=20),
                     '*CONSTRAINED_SPOTWELD_ID', row(17), row(3, 10), row(18), row(5, 8), '*END']
        self.base_raw = raw.replace(b'*END', ('\n'.join(additions)).encode())
        self.rebuild(self.base_raw)
        self.ref = self.directory / 'reference.json'

    def compile_assembly(self, part_ids=(17, 18), **kwargs):
        self.ref.write_text(json.dumps(self.reference))
        with patch.object(yaris_part, 'REFERENCE', self.ref), patch.object(yaris_assembly, 'AUTHENTICATION_PART_ID', 17):
            return compile_archive_assembly(self.archive, self.directory, part_ids, **kwargs)

    def replace_source_only(self, raw):
        """Rehash an authored bad source without hiding it behind importer validation."""
        self.raw = raw
        with zipfile.ZipFile(self.archive, 'w') as archive:
            archive.writestr('fixture/yaris-coarse-v1l.key', raw)
            archive.writestr('fixture/README.md', README)
        self.reference['archive_sha256'] = file_sha256(self.archive)
        self.reference['files']['yaris-coarse-v1l.key']['sha256'] = sha256(raw)
        self.manifest['source_archive']['sha256'] = self.reference['archive_sha256']
        self.manifest['source_files']['yaris-coarse-v1l.key']['sha256'] = sha256(raw)
        self.save()


class SourceAssembly(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.f = AssemblyFixture(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def test_whole_parts_material_tables_and_exact_frontier(self):
        result = self.f.compile_assembly()
        self.assertEqual(result['counts'], dict(parts=2, shells=4, nodes=9, q4=1, native_t3=3,
                                               shared_nodes=0, materials=2, sections=2, curves=2,
                                               internal_nodal_rigid_groups=1, outgoing_nodal_rigid_groups=1,
                                               internal_spotwelds=1, outgoing_spotwelds=1, external_nodes=2))
        self.assertEqual([s['source_elform'] for s in result['declarations']['sections']], [2, 16])
        self.assertEqual([s['thickness_m'][0] for s in result['declarations']['sections']], [.001648, .002])
        self.assertEqual([len(c['stress_pa']) for c in result['declarations']['curves']], [2, 3])
        self.assertEqual(result['attachments']['external_node_ids'], [9, 10])
        self.assertEqual([p['part_id'] for p in result['attachments']['external_parts']], [19])
        self.assertEqual([s['source_id'] for s in result['geometry']['parts'][1]['shells']], [201, 202])
        self.assertEqual(result['geometry']['parts'][1]['shells'][-1]['raw_record'], (202, 18, 7, 8, 16, 16))
        self.assertFalse(result['simulation_ready'])
        self.assertFalse(result['full_attachment_closure_qualified'])
        self.assertFalse(result['mechanics_capacity_changed'])
        self.assertFalse(result['native_mass_ledger']['additional_mass_assigned'])
        self.assertFalse(result['attachments']['tied_candidates'][0]['actual_pairing_qualified'])
        last = result['parent_bindings'][-1]
        self.assertEqual((last['source_element_id'], last['source_material_id'], last['source_section_id'],
                          last['source_curve_id'], last['source_elform']), (202, 38, 28, 48, 16))
        self.assertEqual((last['family'], last['family_index'], last['material_index'], last['curve_index']),
                         ('T3', 2, 1, 1))
        self.assertEqual(last['node_indices'], [6, 7, 8])

    def test_boundary_release_is_declared_and_does_not_delete_source_interfaces(self):
        base = self.f.compile_assembly()
        released = self.f.compile_assembly(boundary_policy='released_external_connections')
        self.assertEqual(base['attachments'], released['attachments'])
        self.assertEqual(base['geometry'], released['geometry'])
        self.assertEqual(released['boundary']['outgoing_nodal_rigid_ids'], [18])
        self.assertEqual(released['boundary']['outgoing_spotweld_ids'], [17])
        self.assertFalse(released['boundary']['applied_to_dynamics'])
        self.assertFalse(released['boundary']['mass_removed_from_selected_parts'])
        self.assertEqual(released['attachments']['tied_candidates'][0]['candidate_pair_scope'], 'may_include_internal_pairs')
        self.assertEqual(released['boundary']['released_external_tied_source_scopes'], [])
        with self.assertRaisesRegex(ValueError, 'boundary policy'):
            self.f.compile_assembly(boundary_policy='silently_fix_nodes')

    def test_elform16_only_selection_does_not_include_authentication_anchor(self):
        r = self.f.compile_assembly(part_ids=(18,))
        self.assertEqual(r['selected_part_ids'], [18])
        self.assertEqual((r['counts']['parts'], r['counts']['shells'], r['counts']['nodes']), (1, 2, 4))
        self.assertEqual(r['declarations']['sections'][0]['source_elform'], 16)
        self.assertEqual({p['source_part_id'] for p in r['parent_bindings']}, {18})
        self.assertEqual(r['source']['authentication_anchor_part_id'], 17)
        self.assertFalse(r['source']['authentication_anchor_automatically_selected'])
        released = self.f.compile_assembly(part_ids=(18,), boundary_policy='released_external_connections')
        self.assertEqual(released['attachments']['tied_candidates'][0]['candidate_pair_scope'], 'cross_boundary_only')
        self.assertEqual(len(released['boundary']['released_external_tied_source_scopes']), 1)
        self.assertEqual(released['boundary']['released_external_tied_source_scopes'][0]['selected_master_part_ids'], [])
        self.assertFalse(released['boundary']['released_external_tied_source_scopes'][0]['actual_pairing_qualified'])
        with patch.object(yaris_part, '_compile_archive_part') as authenticate:
            for part_ids in ((), (18, 18), (True,), tuple(range(1, 10))):
                with self.assertRaisesRegex(ValueError, 'part IDs'):
                    self.f.compile_assembly(part_ids=part_ids)
            authenticate.assert_not_called()

    def test_late_weld_duplicate_missing_endpoint_and_truncated_record_reject(self):
        for replacement, message in [
                (row(17) + '\n' + row(5, 8), 'duplicate spotweld'),
                (row(18) + '\n' + row(5, 999), 'incident node'),
                (row(18), 'incomplete final spotweld')]:
            self.f.rebuild(self.f.base_raw.replace((row(18) + '\n' + row(5, 8)).encode(), replacement.encode()))
            with self.assertRaisesRegex(ValueError, message):
                self.f.compile_assembly()
        self.f.rebuild(self.f.base_raw)
        self.assertEqual(self.f.compile_assembly()['counts']['shells'], 4)

    def test_late_material_section_and_geometry_fail_without_publication(self):
        path = self.f.directory / 'rejected.json'
        for old, new, message in [
                (row(18, 28, 38), row(18, 28, 999), 'unresolved material'),
                (row(28, 16, None, 3), row(28, 12, None, 3), 'ELFORM2/16'),
                (row(18, 28, 38), row(18, 999, 38), 'unresolved section')]:
            self.f.replace_source_only(self.f.base_raw.replace(old.encode(), new.encode()))
            with self.assertRaisesRegex(ValueError, message):
                write_assembly_report(path, self.f.compile_assembly())
            self.assertFalse(path.exists())
        self.f.rebuild(self.f.base_raw)
        original = (self.f.directory / 'node_positions.bin').read_bytes()
        data = bytearray(original)
        struct.pack_into('<d', data, 15 * 3 * 8, 123.)  # Last selected node16.
        self.f.update_bytes('node_positions', data)
        with self.assertRaisesRegex(ValueError, 'pinned source card'):
            write_assembly_report(path, self.f.compile_assembly())
        self.assertFalse(path.exists())

    def test_each_offline_cap_rejects_and_successful_retry_is_identical(self):
        expected = self.f.compile_assembly()
        for changes in ({'parts': 1}, {'shells': 3}, {'nodes': 8}, {'groups': 1},
                        {'spotwelds': 1}, {'frontier_nodes': 1}, {'incident_elements': 1}):
            with self.assertRaisesRegex(ValueError, 'cap'):
                self.f.compile_assembly(limits=replace(AssemblyLimits(), **changes))
        self.assertEqual(expected, self.f.compile_assembly())
        for changes in ({'shells': 1025}, {'nodes': 2049}, {'parts': True}):
            with self.assertRaisesRegex(ValueError, 'cap'):
                AssemblyLimits(**changes)

    def test_late_duplicate_part_and_referenced_unsupported_set_reject(self):
        for addition, message in [('*PART\nlate duplicate\n' + row(18, 28, 38), 'duplicate part'),
                                  ('*SET_NODE_ADD\n' + row(99) + '\n' + row(17) +
                                   '\n*CONSTRAINED_NODAL_RIGID_BODY\n' + row(99, None, 99),
                                   'unsupported set operator')]:
            self.f.replace_source_only(self.f.base_raw.replace(b'*END', (addition + '\n*END').encode()))
            with self.assertRaisesRegex(ValueError, message):
                self.f.compile_assembly()

    def test_original_single_part_output_bytes_and_create_only_publication(self):
        self.f.ref.write_text(json.dumps(self.f.reference))
        with patch.object(yaris_part, 'REFERENCE', self.f.ref):
            before = json.dumps(yaris_part.compile_archive_part(self.f.archive, 17), sort_keys=True).encode()
            report = self.f.compile_assembly()
            after = json.dumps(yaris_part.compile_archive_part(self.f.archive, 17), sort_keys=True).encode()
        self.assertEqual(before, after)
        output = self.f.directory / 'assembly.json'
        write_assembly_report(output, report)
        accepted = output.read_bytes()
        with self.assertRaises(FileExistsError):
            write_assembly_report(output, report)
        self.assertEqual(output.read_bytes(), accepted)


class SpotweldRecords(unittest.TestCase):
    def test_all_records_namespaces_optional_blanks_and_late_duplicate(self):
        raw = ('*KEYWORD\n*CONSTRAINED_SPOTWELD_ID\n' + row(7) + '\n' + row(1, 2, None, 4) +
               '\n$ comment between records\n' + row(8) + '\n' + row(3, 4) + '\n*END\n').encode()
        result = scan_spotwelds(BytesIO(raw), 'authored.key')
        self.assertEqual([w.identity for w in result.records], [7, 8])
        self.assertEqual(result.records[-1].cards[1].blank_mask, 252)
        self.assertEqual(result.records[0].cards[1].fields[3].strip(), '4')
        self.assertFalse(result.records[0].mechanics_qualified)
        self.assertEqual(result.records[-1].cards[0].source_line, 6)
        with self.assertRaisesRegex(ValueError, 'record cap'):
            scan_spotwelds(BytesIO(raw), 'authored.key', record_cap=1)
        with self.assertRaisesRegex(ValueError, 'duplicate spotweld'):
            scan_spotwelds(BytesIO(raw.replace(row(8).encode(), row(7).encode())), 'authored.key')


if __name__ == '__main__':
    unittest.main()
