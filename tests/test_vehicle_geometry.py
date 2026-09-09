"""Small parser/representation regressions without downloading model fixtures."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP / 'tools'))
import import_yaris_vehicle as vehicle


def integers(values, width=8):
    return ''.join(' ' * width if v is None else str(v).rjust(width) for v in values)


def node(nid, xyz, tc=None, rc=None):
    return str(nid).rjust(8) + ''.join(' ' * 16 if v is None else str(v).rjust(16) for v in xyz) + integers([tc, rc])


def fixture():
    lines = ['*KEYWORD', '*TITLE', 'Small geometry test']
    for pid, sid, kind in [(100, 101, 'SHELL'), (200, 102, 'SOLID'), (300, 103, 'BEAM')]:
        lines += ['*PART', f'part_{pid}', integers([pid, sid, 900], 10),
                  '*SECTION_' + kind, integers([sid, 2 if kind == 'SHELL' else 1], 10)]
    lines += ['*MAT_ELASTIC', integers([900], 10), '*CONTACT_AUTOMATIC_SINGLE_SURFACE', 'uninterpreted card',
              '*ELEMENT_SHELL', integers([1001, 100, 1, 2, 3, 3]),
              integers([1002, 100, 1, 2, 3, 4]),
              '*ELEMENT_SOLID', integers([2001, 200, 1, 2, 3, 4, 5, 6, 7, 8]),
              '*ELEMENT_BEAM', integers([3001, 300, 1, 8, None, 1, None, 1, None, 2]), '*NODE']
    for nid in range(1, 9):
        lines.append(node(nid, (nid * 1000, -nid * 1000, 500), tc=7 if nid == 1 else None))
    lines += ['*END']
    return ('\n'.join(lines) + '\n').encode('ascii')


def parsed(data=None):
    geometry = vehicle.VehicleGeometry()
    summary = vehicle.scan(io.BytesIO(fixture() if data is None else data), 'tiny.key', geometry)
    return geometry, summary


class VehicleGeometryUnit(unittest.TestCase):
    def test_ids_units_source_order_and_compact_indices(self):
        geometry, summary = parsed()
        patterns = geometry.validate()
        self.assertEqual(list(geometry.node_ids), list(range(1, 9)))
        self.assertEqual(list(geometry.positions[:3]), [1.0, -1.0, 0.5])
        self.assertEqual(list(geometry.positions[-3:]), [8.0, -8.0, 0.5])
        self.assertEqual(list(geometry.connectivity['shells']), [0, 1, 2, 2, 0, 1, 2, 3])
        self.assertEqual(patterns['shells'], {'3': 1, '4': 1})
        self.assertEqual(list(geometry.node_codes[:2]), [7, 0])
        self.assertGreater(geometry.node_masks[0] & (1 << 5), 0)
        self.assertEqual(summary['sha256'], hashlib.sha256(fixture()).hexdigest())

    def test_repeated_solid_slots_and_beam_raw_options_preserved(self):
        data = fixture().replace(integers([2001, 200, 1, 2, 3, 4, 5, 6, 7, 8]).encode(),
                                 integers([2001, 200, 1, 2, 3, 3, 5, 6, 7, 7]).encode())
        geometry, _ = parsed(data)
        self.assertEqual(geometry.validate()['solids'], {'6': 1})
        self.assertEqual(list(geometry.elements['solids']), [2001, 200, 1, 2, 3, 3, 5, 6, 7, 7])
        self.assertEqual(list(geometry.connectivity['solids']), [0, 1, 2, 2, 4, 5, 6, 6])
        self.assertEqual(list(geometry.elements['beams']), [3001, 300, 1, 8, 0, 1, 0, 1, 0, 2])
        self.assertEqual(geometry.element_masks['beams'][0], (1 << 4) | (1 << 6) | (1 << 8))
        self.assertEqual(list(geometry.connectivity['beams']), [0, 7])

    def test_blank_coordinate_mapping_is_explicit(self):
        data = fixture().replace(node(1, (1000, -1000, 500), tc=7).encode(),
                                 node(1, (None, None, None)).encode())
        geometry, _ = parsed(data)
        geometry.validate()
        self.assertEqual(list(geometry.positions[:3]), [0, 0, 0])
        self.assertEqual(geometry.blank_coordinate_nodes[0]['source_node_id'], 1)
        self.assertEqual(geometry.blank_coordinate_nodes[0]['xyz_blank_mask'], 7)

    def test_part_section_material_references_are_metadata_only(self):
        geometry, summary = parsed()
        geometry.validate()
        self.assertEqual(geometry.parts[100]['source_section_id'], 101)
        self.assertEqual(geometry.parts[100]['source_material_id'], 900)
        self.assertEqual(geometry.parts[100]['title'], 'part_100')
        self.assertEqual(geometry.sections[101]['formulation_field_raw'], '2')
        self.assertEqual(geometry.materials[900]['mechanics_status'], 'BLOCKING')
        for block in summary['blocks']:
            if block['keyword'].startswith(('*MAT_', '*SECTION_', '*CONTACT_')):
                self.assertEqual(block['disposition'], 'BLOCKING')
            self.assertNotEqual(block['disposition'], 'SUPPORTED')

    def test_unknown_keywords_and_setup_geometry_remain_blocking(self):
        data = fixture().replace(b'*END', b'*UNIMPLEMENTED_NEW_PHYSICS\n1\n*END')
        _, summary = parsed(data)
        self.assertEqual(summary['blocks'][-2]['disposition'], 'BLOCKING')
        other = vehicle.scan(io.BytesIO(fixture()), 'setup.key')
        for block in other['blocks']:
            if block['keyword'] in vehicle.GEOMETRY_KEYWORDS:
                self.assertEqual(block['disposition'], 'BLOCKING')
        manifest = dict(simulation_ready=False, source_files={'setup.key': other})
        with self.assertRaisesRegex(vehicle.WallImportError, 'geometry-only'):
            vehicle.assert_simulation_ready(manifest)
        manifest['simulation_ready'] = True
        with self.assertRaisesRegex(vehicle.WallImportError, 'unresolved'):
            vehicle.assert_simulation_ready(manifest)

    def test_keyword_block_hashes_and_source_lines(self):
        data = fixture().replace(b'*NODE\n', b'$ inactive *NODE\n*NODE\n$ node comment\n')
        geometry, summary = parsed(data)
        self.assertEqual(summary['keyword_counts']['*NODE'], 1)
        lines = data.splitlines(keepends=True)
        for block in summary['blocks']:
            original = b''.join(lines[block['first_line'] - 1:block['last_line']])
            self.assertEqual(hashlib.sha256(original).hexdigest(), block['source_block_sha256'])
        for nid, line in zip(geometry.node_ids, geometry.node_lines):
            self.assertEqual(int(lines[line - 1][:8]), nid)

    def test_duplicate_missing_and_malformed_geometry_fail(self):
        original_node = node(1, (1000, -1000, 500), tc=7).encode()
        cases = [fixture().replace(original_node, original_node + b'\n' + original_node),
                 fixture().replace(original_node, node(1, ('NaN', 0, 0)).encode()),
                 fixture().replace(original_node, node(1, ('bad', 0, 0)).encode()),
                 fixture().replace(b'*END', b''),
                 fixture().replace(b'*KEYWORD\n', b''),
                 fixture() + b'*NODE\n',
                 fixture().replace(integers([1001, 100, 1, 2, 3, 3]).encode(),
                                   integers([1001, 100, 1, 2, 2, 2]).encode()),
                 fixture().replace(integers([3001, 300, 1, 8, None, 1, None, 1, None, 2]).encode(),
                                   integers([1001, 300, 1, 8, None, 1, None, 1, None, 2]).encode())]
        for i, data in enumerate(cases):
            with self.subTest(case=i), self.assertRaises(vehicle.WallImportError):
                parsed(data)

    def test_unresolved_node_part_section_material_fail(self):
        cases = [fixture().replace(node(8, (8000, -8000, 500)).encode() + b'\n', b''),
                 fixture().replace(integers([1001, 100, 1, 2, 3, 3]).encode(),
                                   integers([1001, 999, 1, 2, 3, 3]).encode()),
                 fixture().replace(integers([100, 101, 900], 10).encode(), integers([100, 999, 900], 10).encode()),
                 fixture().replace(integers([100, 101, 900], 10).encode(), integers([100, 101, 999], 10).encode())]
        for i, data in enumerate(cases):
            with self.subTest(case=i), self.assertRaises(vehicle.WallImportError):
                geometry, _ = parsed(data)
                geometry.validate()

    def test_little_endian_arrays_independently_decode(self):
        geometry, _ = parsed()
        geometry.validate()
        with tempfile.TemporaryDirectory(prefix='vehicle-array-') as tmp:
            path = Path(tmp) / 'positions.bin'
            descriptor = vehicle.write_array(path, geometry.positions, 3, ['x', 'y', 'z'])
            self.assertEqual(descriptor['dtype'], '<f8')
            self.assertEqual(descriptor['shape'], [8, 3])
            self.assertEqual(struct.unpack_from('<ddd', path.read_bytes()), (1, -1, 0.5))
            self.assertEqual(descriptor['sha256'], hashlib.sha256(path.read_bytes()).hexdigest())
            path = Path(tmp) / 'shells.bin'
            vehicle.write_array(path, geometry.elements['shells'], 6)
            self.assertEqual(struct.unpack_from('<6Q', path.read_bytes()), (1001, 100, 1, 2, 3, 3))

    def test_wrong_archive_pin_publishes_nothing(self):
        with tempfile.TemporaryDirectory(prefix='vehicle-pin-') as tmp:
            bad = Path(tmp) / 'wrong.zip'
            bad.write_bytes(b'not the source model')
            output = Path(tmp) / 'result'
            with self.assertRaisesRegex(vehicle.WallImportError, 'SHA256'):
                vehicle.compile_archive(bad, output)
            self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()
