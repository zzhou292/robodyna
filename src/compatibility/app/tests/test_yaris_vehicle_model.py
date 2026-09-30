"""Full pinned artifact gate; requires explicit prepared model assets, no skip.

The standard tiny suite is separate. Set YARIS_VEHICLE_ASSET_DIR if assets are
outside the workspace default. This test does not run a solver or compiler.
"""
from array import array
from collections import Counter
from decimal import Decimal
import hashlib
import json
import os
from pathlib import Path
import sys
import unittest
import zipfile

APP = Path(__file__).resolve().parents[1]
DEFAULT_ASSETS = APP.parent / 'crash-work/assets/yaris-vehicle'
ASSETS = Path(os.environ.get('YARIS_VEHICLE_ASSET_DIR', DEFAULT_ASSETS))
sys.path.insert(0, str(APP / 'tools'))
import import_yaris_vehicle as vehicle


def hash_file(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(data)
    return digest.hexdigest()


class YarisVehicleArtifacts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = json.loads((ASSETS / 'manifest.json').read_text())
        cls.reference = json.loads((APP / 'models/yaris_coarse_v1l.json').read_text())

    def load_array(self, name, typecode):
        descriptor = self.manifest['arrays'][name]
        result = array(typecode)
        with (ASSETS / descriptor['file']).open('rb') as stream:
            result.fromfile(stream, descriptor['shape'][0] * descriptor['shape'][1])
            self.assertEqual(stream.read(1), b'')
        if sys.byteorder != 'little':
            result.byteswap()
        return result

    def test_counts_complete_source_census_and_no_physics_admission(self):
        self.assertEqual(self.manifest['counts'], dict(nodes=393165, parts=919, shells=358457,
                                                     solids=15234, beams=4685))
        self.assertEqual(self.manifest['unique_node_patterns']['shells'], {'3': 21481, '4': 336976})
        self.assertEqual(self.manifest['unique_node_patterns']['solids'], {'6': 1388, '8': 13846})
        self.assertFalse(self.manifest['simulation_ready'])
        self.assertFalse(self.manifest['m1_complete'])
        self.assertIsNone(self.manifest['applied_rigid_transform'])
        self.assertEqual(self.manifest['source_coordinate_frame'],
                         'Untransformed original yaris-coarse-v1l.key coordinates')
        with self.assertRaises(vehicle.WallImportError):
            vehicle.assert_simulation_ready(self.manifest)
        all_keywords = set()
        for name, expected in self.reference['files'].items():
            summary = self.manifest['source_files'][name]
            self.assertEqual(summary['sha256'], expected['sha256'])
            self.assertEqual(summary['keyword_counts'], expected['keyword_counts'])
            all_keywords.update(summary['keyword_counts'])
            for block in summary['blocks']:
                self.assertNotEqual(block['disposition'], 'SUPPORTED')
                if block['keyword'].startswith(('*MAT_', '*SECTION_', '*CONSTRAINED_', '*AIRBAG_', '*CONTACT_')):
                    self.assertEqual(block['disposition'], 'BLOCKING')
        self.assertEqual(len(all_keywords), 84)

    def test_array_and_source_hashes_and_layouts(self):
        self.assertEqual(hash_file(ASSETS / 'source_model.zip'), self.reference['archive_sha256'])
        self.assertEqual(self.manifest['generator']['sha256'], hash_file(APP / 'tools/import_yaris_vehicle.py'))
        self.assertEqual(self.manifest['generator']['shared_wall_utilities_sha256'], hash_file(APP / 'tools/import_yaris_wall.py'))
        self.assertEqual(self.manifest['generator']['model_reference_sha256'], hash_file(APP / 'models/yaris_coarse_v1l.json'))
        for descriptor in self.manifest['arrays'].values():
            path = ASSETS / descriptor['file']
            size = descriptor['shape'][0] * descriptor['shape'][1] * int(descriptor['dtype'][2:])
            self.assertEqual(path.stat().st_size, size)
            self.assertEqual(path.stat().st_size, descriptor['bytes'])
            self.assertEqual(hash_file(path), descriptor['sha256'])
        for line in (ASSETS / 'SHA256SUMS').read_text().splitlines():
            digest, name = line.split('  ', 1)
            self.assertEqual(hash_file(ASSETS / name), digest)

    def test_every_geometry_record_matches_original_and_indices_round_trip(self):
        ids = self.load_array('node_ids', 'Q')
        positions = self.load_array('node_positions', 'd')
        source_lines = self.load_array('node_source_lines', 'I')
        tables = {name: self.load_array(name + '_records', 'Q') for name in ('shells', 'solids', 'beams')}
        indices = {name: self.load_array(name + '_node_indices', 'I') for name in tables}
        line_tables = {name: self.load_array(name + '_source_lines', 'I') for name in tables}
        expected_widths = {'shells': 6, 'solids': 10, 'beams': 10}
        source_keywords = {'*ELEMENT_SHELL': 'shells', '*ELEMENT_SOLID': 'solids', '*ELEMENT_BEAM': 'beams'}
        counts = Counter()
        with zipfile.ZipFile(ASSETS / 'source_model.zip') as archive:
            with archive.open('2010-toyota-yaris-coarse-v1l/yaris-coarse-v1l.key') as source:
                active = None
                for number, raw in enumerate(source, 1):
                    line = raw.decode('ascii').strip()
                    if line.startswith('*'):
                        active = line
                        continue
                    if not line or line.startswith('$'):
                        continue
                    if active == '*NODE':
                        row = counts['nodes']
                        tokens = line.split()
                        self.assertEqual(ids[row], int(tokens[0]))
                        self.assertEqual(source_lines[row], number)
                        # All coordinates use an independent Decimal /1000 conversion.
                        xyz = [Decimal(x) / 1000 for x in tokens[1:4]] if len(tokens) >= 4 else [Decimal(0)] * 3
                        for axis in range(3):
                            self.assertAlmostEqual(positions[3 * row + axis], float(xyz[axis]), delta=1e-12)
                        counts['nodes'] += 1
                    elif active in source_keywords:
                        name = source_keywords[active]
                        row, width = counts[name], expected_widths[name]
                        if name == 'beams':
                            text = raw.decode('ascii').rstrip('\r\n')
                            original = [int(text[i * 8:(i + 1) * 8].strip() or 0) for i in range(10)]
                        else:
                            original = [int(x) for x in line.split()]
                        self.assertEqual(list(tables[name][row * width:(row + 1) * width]), original)
                        self.assertEqual(line_tables[name][row], number)
                        connectivity_width = 2 if name == 'beams' else width - 2
                        mapped = [ids[index] for index in indices[name][row * connectivity_width:(row + 1) * connectivity_width]]
                        self.assertEqual(mapped, original[2:2 + connectivity_width])
                        counts[name] += 1
        self.assertEqual(dict(counts), {name: self.manifest['counts'][name] for name in ('nodes', 'shells', 'solids', 'beams')})
        self.assertEqual(ids[0], 2000001)
        self.assertEqual(list(positions[:3]), [0, 0, 0])
        self.assertEqual(self.manifest['blank_coordinate_mapping']['affected_nodes'],
                         [{'source_node_id': 2000001, 'source_line': 409491, 'xyz_blank_mask': 7}])


if __name__ == '__main__':
    unittest.main()
