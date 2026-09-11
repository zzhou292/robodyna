"""Tiny source-authenticated solid export controls; no original vehicle parse."""
from array import array
from copy import deepcopy
from io import BytesIO
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
sys.path.insert(0, str(APP / 'tools'))
import import_yaris_vehicle as vehicle
from modelio.canonical_geometry import file_hash, load_array, _bits
from modelio.solid_working_geometry import select_geometry, collect_working_solids
from modelio.solid_geometry_export import export_geometry


def row(*values, width=10):
    return ''.join(str(v).rjust(width) if v is not None else ' ' * width for v in values)


class TinySolid:
    def __init__(self, root):
        self.root = Path(root)
        self.assets = self.root / 'canonical'
        self.assets.mkdir()
        self.source = self.root / 'source.key'
        self.output = self.root / 'export'
        self.reference = self.root / 'reference.json'
        lines = ['*KEYWORD', '*PART', '$ title comment', 'tiny foam', row(17, 27, 37),
                 '*SECTION_SOLID', row(27, 2), '*MAT_LOW_DENSITY_FOAM',
                 row(37, '7.7200E-10', 21, 47, 15), row(None, 0, None, None, None, 20000),
                 '*DEFINE_CURVE', row(47, 0, 1, 1), row(0, 0, width=20), row(.1, 3.44, width=20),
                 '*ELEMENT_SOLID', row(202, 17, 9, 3, 6, 1, 8, 2, 7, 4, width=8),
                 row(101, 17, 3, 9, 6, 1, 2, 2, 7, 7, width=8), '*NODE']
        nids = (9, 3, 6, 1, 8, 2, 7, 4)
        xyz = (('-0.0', 0, 0), ('1000.1234', 0, 0), (1000, 1000, 0), (None, 1000, 0),
               (0, 0, 1000), (1000, 0, 1000), (1000, 1000, 1000), (0, 1000, 1000))
        for nid, position in zip(nids, xyz):
            lines.append(row(nid, width=8) + row(*position, width=16) +
                         row(-7 if nid == 9 else None, 0 if nid == 9 else None, width=8))
        lines.append('*END')
        self.raw = ('\n'.join(lines) + '\n').encode('ascii')
        self.source.write_bytes(self.raw)
        geometry = vehicle.VehicleGeometry()
        summary = vehicle.scan(BytesIO(self.raw), 'yaris-coarse-v1l.key', geometry)
        geometry.validate()
        self.ref = dict(archive_sha256='a'*64, archive_member_prefix='original/',
                        files={'yaris-coarse-v1l.key': {'sha256': vehicle.sha256(self.raw)}})
        self.reference.write_text(json.dumps(self.ref))
        self.manifest = dict(schema='tlfea.yaris_source_vehicle_geometry.v1', simulation_ready=False,
                             canonical_geometry_validated=True, applied_rigid_transform=None,
                             source_length_unit='mm', output_length_unit='m', length_scale=.001,
                             source_archive={'sha256': self.ref['archive_sha256']},
                             source_files={'yaris-coarse-v1l.key': summary},
                             parts=list(geometry.parts.values()), arrays={})
        values = {'node_ids': (geometry.node_ids, 1), 'node_positions': (geometry.positions, 3),
                  'node_codes': (geometry.node_codes, 2), 'node_blank_masks': (geometry.node_masks, 1),
                  'node_source_lines': (geometry.node_lines, 1),
                  'solids_records': (geometry.elements['solids'], 10),
                  'solids_node_indices': (geometry.connectivity['solids'], 8),
                  'solids_source_lines': (geometry.element_lines['solids'], 1),
                  'solids_blank_masks': (geometry.element_masks['solids'], 1)}
        for name, (data, columns) in values.items():
            self.put_array(name, data, columns)
        self.save()

    def put_array(self, name, data, columns):
        path = self.assets / (name + '.bin')
        self.manifest['arrays'][name] = dict(file=path.name, **vehicle.write_array(path, data, columns, None))

    def save(self):
        (self.assets / 'manifest.json').write_text(json.dumps(self.manifest))

    def select(self, **kwargs):
        return select_geometry(self.assets, [17], file_hash(self.assets / 'manifest.json'), **kwargs)

    def collect(self, raw=None):
        _, nodes, elements, _ = self.select()
        return collect_working_solids(BytesIO(self.raw if raw is None else raw), nodes, elements, [17])

    def export(self, **kwargs):
        return export_geometry(self.assets, self.source, self.output, [17],
                               file_hash(self.assets / 'manifest.json'), reference_path=self.reference, **kwargs)


class SolidWorkingGeometry(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.f = TinySolid(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def test_source_slots_signed_zero_native_coordinates_and_exact_caps(self):
        _, nodes, elements, _ = self.f.select(parent_cap=2, node_cap=8)
        working, solids, _ = self.f.collect()
        self.assertEqual(list(nodes), [9, 3, 6, 1, 8, 2, 7, 4])
        self.assertEqual(list(elements), [202, 101])
        self.assertEqual(solids[101]['raw_record'], (101, 17, 3, 9, 6, 1, 2, 2, 7, 7))
        self.assertEqual(working[9]['codes'], (-7, 0))
        self.assertEqual(working[1]['blank_mask'] & 2, 2)
        self.assertEqual(_bits(working[9]['position_native'][:1]), struct.pack('<d', -0.0))
        self.assertEqual(_bits(nodes[3]['position_m']), _bits((1000.1234*.001, 0., 0.)))
        self.assertEqual(working[3]['position_native'][0], 1000.1234)
        self.assertNotEqual(working[3]['position_native'][0], nodes[3]['position_m'][0]/.001)
        with self.assertRaisesRegex(ValueError, 'extent'):
            self.f.select(parent_cap=1)
        with self.assertRaisesRegex(ValueError, 'node cap'):
            self.f.select(node_cap=7)

    def test_cap_identity_and_parallel_array_rejections_precede_unsafe_reads(self):
        with patch('modelio.solid_working_geometry.load_array', side_effect=AssertionError('unexpected load')):
            with self.assertRaisesRegex(ValueError, 'cap'):
                self.f.select(parent_cap=0)
            self.f.manifest['arrays']['node_ids']['bytes'] = 64*1024*1024 + 1
            self.f.save()
            with self.assertRaisesRegex(ValueError, 'array bytes'):
                self.f.select()
        self.f.put_array('node_codes', array('i', [0, 0]), 2)
        self.f.put_array('node_ids', array('Q', [9, 3, 6, 1, 8, 2, 7, 4]), 1)
        self.f.save()
        with self.assertRaisesRegex(ValueError, 'parallel'):
            self.f.select()

    def test_rehashed_index_coordinate_and_duplicate_id_lies_reject(self):
        before = deepcopy(self.f.manifest)
        path = self.f.assets / 'solids_node_indices.bin'
        original = path.read_bytes()
        data = array('I'); data.frombytes(original); data[0] = 1
        self.f.put_array('solids_node_indices', data, 8); self.f.save()
        with self.assertRaisesRegex(ValueError, 'association'):
            self.f.select()
        path.write_bytes(original); self.f.manifest = before; self.f.save()
        positions = array('d'); positions.frombytes((self.f.assets/'node_positions.bin').read_bytes())
        positions[0] = 0.0  # Equal numerically; different original signed-zero bits.
        self.f.put_array('node_positions', positions, 3); self.f.save()
        with self.assertRaisesRegex(ValueError, 'NODE/canonical'):
            self.f.collect()
        self.f.put_array('node_ids', array('Q', [9, 9, 6, 1, 8, 2, 7, 4]), 1); self.f.save()
        with self.assertRaisesRegex(ValueError, 'duplicate'):
            self.f.select()

    def test_late_duplicate_missing_extra_and_code_mismatch_preserve_retry(self):
        baseline = self.f.collect()[:2]
        lines = self.f.raw.decode().splitlines()
        variants = [lines[:-1] + [lines[-2], '*END'],
                    lines[:-2] + ['*END'],
                    lines[:-1] + ['*ELEMENT_SOLID', row(999, 17, 9, 3, 6, 1, 8, 2, 7, 4, width=8), '*END']]
        changed = lines.copy(); changed[18] = changed[18][:-8] + row(1, width=8)
        variants.append(changed)
        for changed in variants:
            with self.assertRaises(ValueError):
                self.f.collect(('\n'.join(changed)+'\n').encode())
        self.assertEqual(baseline, self.f.collect()[:2])

    def test_binary_export_exact_raw_blocks_density_bits_and_source_order(self):
        report = self.f.export(expected_parents=2, expected_nodes=8)
        def read(name, code, dtype, width):
            return load_array(self.f.output, report, name, code, dtype, width)
        self.assertEqual(tuple(read('solid_records_u64', 'Q', '<u8', 10)[10:]),
                         (101, 17, 3, 9, 6, 1, 2, 2, 7, 7))
        self.assertEqual(tuple(read('solid_nodes_local_u32', 'I', '<u4', 8)[8:]), (1, 0, 2, 3, 5, 5, 6, 6))
        self.assertEqual(read('node_position_mm_f64', 'd', '<f8', 3)[3], 1000.1234)
        self.assertEqual(read('node_codes_i64', 'q', '<i8', 2)[0], -7)
        self.assertEqual(read('node_blank_mask_u32', 'I', '<u4', 1)[3] & 2, 2)
        part = report['declarations']['parts'][0]
        self.assertEqual((part['part_id'], part['section_id'], part['material_id'], part['curve_id']), (17, 27, 37, 47))
        self.assertEqual(part['density_kg_per_m3']['binary64_le'], struct.pack('<d', float('7.7200E-10')*1e12).hex())
        self.assertEqual(part['density_source_field'], '7.7200E-10'.rjust(10))
        for block in report['declarations']['blocks']:
            self.assertEqual(vehicle.sha256(block['raw_text'].encode()), block['sha256'])
        self.assertIn('$ title comment', next(b['raw_text'] for b in report['declarations']['blocks'] if b['family']=='part'))
        self.assertFalse(report['simulation_ready'])
        self.assertFalse(report['source']['archive_container_verified_by_exporter'])
        for name, descriptor in report['arrays'].items():
            self.assertEqual(file_hash(self.f.output/descriptor['file']), descriptor['sha256'], name)

    def test_publication_cap_counts_provenance_and_create_only(self):
        with self.assertRaisesRegex(ValueError, 'byte cap'):
            self.f.export(byte_cap=1)
        self.assertFalse(self.f.output.exists())
        with self.assertRaisesRegex(ValueError, 'count mismatch'):
            self.f.export(expected_parents=1)
        self.assertFalse(self.f.output.exists())
        self.f.ref['archive_sha256'] = 'b'*64
        self.f.reference.write_text(json.dumps(self.f.ref))
        with self.assertRaisesRegex(ValueError, 'reference identity'):
            self.f.export()
        self.assertFalse(self.f.output.exists())
        self.f.ref['archive_sha256'] = 'a'*64
        self.f.reference.write_text(json.dumps(self.f.ref))
        report = self.f.export()
        old = (self.f.output/'manifest.json').read_bytes()
        with patch('modelio.solid_geometry_export.select_geometry', side_effect=AssertionError('unexpected load')):
            with self.assertRaisesRegex(ValueError, 'already exists'):
                self.f.export()
        self.assertEqual(old, (self.f.output/'manifest.json').read_bytes())
        self.assertEqual(json.loads(old)['solid_count'], report['solid_count'])

    def test_exact_complete_output_byte_cap_and_one_byte_short(self):
        self.f.export()
        size = sum(p.stat().st_size for p in self.f.output.iterdir())
        self.f.output = self.f.root / 'one-byte-short'
        with self.assertRaisesRegex(ValueError, 'byte cap'):
            self.f.export(byte_cap=size-1)
        self.assertFalse(self.f.output.exists())
        self.f.output = self.f.root / 'exact-cap'
        self.f.export(byte_cap=size)
        self.assertEqual(sum(p.stat().st_size for p in self.f.output.iterdir()), size)

    def test_nonfinite_out_of_range_duplicate_eid_and_missing_part(self):
        positions = array('d'); positions.frombytes((self.f.assets/'node_positions.bin').read_bytes())
        original = array('d', positions)
        positions[-1] = float('inf')
        self.f.put_array('node_positions', positions, 3); self.f.save()
        with self.assertRaisesRegex(ValueError, 'nonfinite'):
            self.f.select()
        self.f.put_array('node_positions', original, 3)
        indices = array('I'); indices.frombytes((self.f.assets/'solids_node_indices.bin').read_bytes())
        before = array('I', indices); indices[-1] = 999
        self.f.put_array('solids_node_indices', indices, 8); self.f.save()
        with self.assertRaisesRegex(ValueError, 'out of range'):
            self.f.select()
        self.f.put_array('solids_node_indices', before, 8)
        records = array('Q'); records.frombytes((self.f.assets/'solids_records.bin').read_bytes())
        records[10] = records[0]
        self.f.put_array('solids_records', records, 10); self.f.save()
        with self.assertRaisesRegex(ValueError, 'extent/identity'):
            self.f.select()
        records[10] = 101
        self.f.put_array('solids_records', records, 10); self.f.save()
        with self.assertRaisesRegex(ValueError, 'selection'):
            select_geometry(self.f.assets, [17, 18], file_hash(self.f.assets/'manifest.json'))
        with self.assertRaisesRegex(ValueError, 'canonical identity'):
            select_geometry(self.f.assets, [17], '0'*64)

    def test_late_declaration_duplicate_or_rehashed_source_mismatch_no_output(self):
        self.f.source.write_bytes(self.f.raw + b'$ original source changed\n')
        with self.assertRaisesRegex(ValueError, 'SHA256'):
            self.f.export()
        self.assertFalse(self.f.output.exists())
        duplicate = self.f.raw.replace(b'*END', b'*SECTION_SOLID\n'+row(27, 2).encode()+b'\n*END')
        self.f.source.write_bytes(duplicate)
        digest = vehicle.sha256(duplicate)
        self.f.ref['files']['yaris-coarse-v1l.key']['sha256'] = digest
        self.f.reference.write_text(json.dumps(self.f.ref))
        self.f.manifest['source_files']['yaris-coarse-v1l.key']['sha256'] = digest
        self.f.save()
        with self.assertRaisesRegex(ValueError, 'duplicate section'):
            self.f.export()
        self.assertFalse(self.f.output.exists())


if __name__ == '__main__':
    unittest.main()
