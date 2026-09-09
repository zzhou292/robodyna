"""Source-authenticated selection tests on tiny authored fixed-width cards."""
from dataclasses import replace
from io import BytesIO
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import zipfile

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
sys.path.insert(0, str(APP / 'tools'))
from modelio.canonical_geometry import GeometryLimits, load_part_geometry
from modelio.declarations import UnitSystem
from modelio.keyword_cards import compile_part_declarations
from modelio.source_blocks import scan_declarations
import import_yaris_vehicle as vehicle


def row(*values, width=10):
    return ''.join(str(v).rjust(width) if v is not None else ' ' * width for v in values)


class TinyCanonical:
    """Author source cards independently; reuse the existing canonical writer."""
    def __init__(self, directory):
        self.directory = Path(directory)
        lines = ['*KEYWORD', '*PART', 'small connected shell part', row(17, 27, 37),
                 '*SECTION_SHELL', row(27, 2, None, 3), row(1.648, 1.648, 1.648, 1.648),
                 '*MAT_PIECEWISE_LINEAR_PLASTICITY', row(37, '7.8900E-9', '2.0000E+5', .3, 270),
                 row(8000, 8, 47, None, '0.0'), '', '', '*DEFINE_CURVE', row(47, 0, 1, 1),
                 row(0, 270, width=20), row(.3, 362, width=20), '*ELEMENT_SHELL',
                 row(101, 17, 1, 2, 3, 4, width=8), row(102, 17, 2, 5, 3, 3, width=8), '*NODE']
        for nid, xyz in enumerate(((0, 0, 0), (2000, 0, 0), (2000, 1000, 0), (0, 1000, 0), (3000, 0, 0)), 1):
            lines.append(row(nid, width=8) + row(*xyz, width=16) + row(7 if nid == 1 else None, None, width=8))
        lines.append('*END')
        self.raw = ('\n'.join(lines) + '\n').encode('ascii')
        parsed = vehicle.VehicleGeometry()
        summary = vehicle.scan(BytesIO(self.raw), 'yaris-coarse-v1l.key', parsed)
        parsed.validate()
        index = scan_declarations(BytesIO(self.raw), 'yaris-coarse-v1l.key')
        self.declarations = compile_part_declarations(index, 17, UnitSystem('t', 'mm', 's', 1000, .001, 1))
        self.archive = self.directory / 'source.zip'
        self.write_archive(self.raw)
        self.manifest = dict(schema='tlfea.yaris_source_vehicle_geometry.v1', simulation_ready=False,
                             canonical_geometry_validated=True, applied_rigid_transform=None,
                             source_length_unit='mm', output_length_unit='m', length_scale=.001,
                             source_coordinate_frame='Untransformed original yaris-coarse-v1l.key coordinates',
                             source_archive={'sha256': self.reference['archive_sha256']},
                             source_files={'yaris-coarse-v1l.key': summary}, parts=list(parsed.parts.values()), arrays={})
        values = {'node_ids': (parsed.node_ids, 1), 'node_positions': (parsed.positions, 3),
                  'node_codes': (parsed.node_codes, 2), 'node_blank_masks': (parsed.node_masks, 1),
                  'node_source_lines': (parsed.node_lines, 1), 'shells_records': (parsed.elements['shells'], 6),
                  'shells_node_indices': (parsed.connectivity['shells'], 4),
                  'shells_source_lines': (parsed.element_lines['shells'], 1),
                  'shells_blank_masks': (parsed.element_masks['shells'], 1)}
        for name, (data, columns) in values.items():
            filename = name + '.bin'
            self.manifest['arrays'][name] = dict(file=filename, **vehicle.write_array(self.directory / filename, data, columns, None))
        self.save()

    def write_archive(self, raw):
        with zipfile.ZipFile(self.archive, 'w') as archive:
            archive.writestr('fixture/yaris-coarse-v1l.key', raw)
        self.reference = dict(archive_member_prefix='fixture/', archive_sha256=vehicle.file_sha256(self.archive),
                              files={'yaris-coarse-v1l.key': {'sha256': vehicle.sha256(raw)}})

    def save(self):
        (self.directory / 'manifest.json').write_text(json.dumps(self.manifest))

    def update_bytes(self, name, data):
        info = self.manifest['arrays'][name]
        (self.directory / info['file']).write_bytes(data)
        info['sha256'] = vehicle.sha256(data)
        info['bytes'] = len(data)
        self.save()

    def load(self, **kwargs):
        return load_part_geometry(self.directory, self.archive, self.declarations, self.reference, **kwargs)


class CanonicalPartGeometry(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.fixture = TinyCanonical(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def test_exact_source_ids_native_triangle_units_flags_and_immutable_retry(self):
        result = self.fixture.load()
        self.assertEqual([s.source_id for s in result.shells], [101, 102])
        self.assertEqual([s.arity for s in result.shells], [4, 3])
        self.assertEqual(result.shells[1].raw_record, (102, 17, 2, 5, 3, 3))
        self.assertEqual(result.shells[1].local_node_indices, (1, 4, 2, 2))
        self.assertEqual([n.position_m for n in result.nodes], [(0., 0., 0.), (2., 0., 0.), (2., 1., 0.), (0., 1., 0.), (3., 0., 0.)])
        self.assertEqual(result.nodes[0].codes, (7, 0))
        self.assertEqual(result.nodes[0].blank_mask, 1 << 5)
        self.assertEqual(result.nodes[1].blank_mask, (1 << 4) | (1 << 5))
        self.assertEqual(result.shells[0].source_line, 18)
        self.assertEqual(result.nodes[0].source_line, 21)
        with self.assertRaisesRegex(ValueError, 'node cap'):
            self.fixture.load(limits=GeometryLimits(nodes=4))
        self.assertEqual(result, self.fixture.load())

    def test_array_checksum_and_shape_and_missing_record_fail(self):
        f = self.fixture
        path = f.directory / f.manifest['arrays']['node_positions']['file']
        original = path.read_bytes()
        path.write_bytes(original[:-1] + bytes([original[-1] ^ 1]))
        with self.assertRaisesRegex(ValueError, 'checksum'):
            f.load()
        path.write_bytes(original)
        f.manifest['arrays']['node_positions']['shape'][0] += 1
        f.save()
        with self.assertRaisesRegex(ValueError, 'shape/size'):
            f.load()
        del f.manifest['arrays']['node_ids']
        f.save()
        with self.assertRaisesRegex(ValueError, 'metadata'):
            f.load()

    def test_self_consistent_rehashed_coordinate_lie_fails_source_verification(self):
        f = self.fixture
        data = bytearray((f.directory / 'node_positions.bin').read_bytes())
        struct.pack_into('<d', data, 8 * 3, 2.00001)
        f.update_bytes('node_positions', data)
        with self.assertRaisesRegex(ValueError, 'pinned source card'):
            f.load()

    def test_index_id_disagreement_and_out_of_range_reject(self):
        f = self.fixture
        original = (f.directory / 'shells_node_indices.bin').read_bytes()
        for index in (3, 999):
            data = bytearray(original)
            struct.pack_into('<I', data, 0, index)
            f.update_bytes('shells_node_indices', data)
            with self.assertRaisesRegex(ValueError, 'index'):
                f.load()

    def test_omitted_native_triangle_cannot_hide_behind_rehashed_arrays(self):
        f = self.fixture
        for name, width in [('shells_records', 6 * 8), ('shells_node_indices', 4 * 4),
                            ('shells_source_lines', 4), ('shells_blank_masks', 2)]:
            data = (f.directory / (name + '.bin')).read_bytes()[:width]
            f.manifest['arrays'][name]['shape'][0] = 1
            f.update_bytes(name, data)
        with self.assertRaisesRegex(ValueError, 'selected source element'):
            f.load()

    def test_source_line_or_blank_mask_cannot_be_rewritten(self):
        f = self.fixture
        for name, code, value in [('node_source_lines', '<I', 22), ('node_blank_masks', '<H', 0)]:
            original = (f.directory / (name + '.bin')).read_bytes()
            data = bytearray(original)
            struct.pack_into(code, data, 0, value)
            f.update_bytes(name, data)
            with self.assertRaisesRegex(ValueError, 'pinned source card'):
                f.load()
            f.update_bytes(name, original)

    def test_join_transform_duplicate_key_path_and_caps_fail(self):
        f = self.fixture
        old = f.declarations
        f.declarations = replace(old, part=replace(old.part, section_id=28))
        with self.assertRaisesRegex(ValueError, 'join'):
            f.load()
        f.declarations = old
        f.manifest['applied_rigid_transform'] = {'translation': [0, 0, 0]}
        f.save()
        with self.assertRaisesRegex(ValueError, 'original-frame'):
            f.load()
        f.manifest['applied_rigid_transform'] = None
        f.save()
        with self.assertRaisesRegex(ValueError, 'shell cap'):
            f.load(limits=GeometryLimits(shells=1))
        f.manifest['arrays']['node_ids']['file'] = '../elsewhere.bin'
        f.save()
        with self.assertRaisesRegex(ValueError, 'escapes'):
            f.load()
        (f.directory / 'manifest.json').write_text('{"a":1,"a":2}')
        with self.assertRaisesRegex(ValueError, 'duplicate JSON'):
            f.load()
        with self.assertRaises(ValueError):
            GeometryLimits(nodes=513)

    def test_archive_hash_member_coverage_and_malformed_size_fail(self):
        f = self.fixture
        with f.archive.open('ab') as stream:
            stream.write(b'changed')
        with self.assertRaisesRegex(ValueError, 'archive SHA256'):
            f.load()
        f.write_archive(f.raw)
        # Recreating ZIP may change its timestamp: carry the explicit new pin.
        f.manifest['source_archive']['sha256'] = f.reference['archive_sha256']
        f.manifest['arrays']['node_ids']['shape'][0] = True
        f.save()
        with self.assertRaisesRegex(ValueError, 'row count'):
            f.load()


if __name__ == '__main__':
    unittest.main()
