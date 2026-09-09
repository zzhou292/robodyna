"""Independent U02 checks against the original pinned wall snapshots (no GPU)."""
from collections import Counter, defaultdict
from contextlib import redirect_stderr
from decimal import Decimal
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

APP_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = APP_ROOT / 'tools/import_yaris_wall.py'
FIXTURE = APP_ROOT / 'tests/data/yaris-wall'
spec = importlib.util.spec_from_file_location('yaris_wall_importer', SCRIPT)
wall = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wall)


def change_record(text, keyword, record_id, operation):
    lines = text.splitlines()
    active = False
    for i, line in enumerate(lines):
        if line.startswith('*'):
            active = line == keyword
        elif active and line[:8].strip() == str(record_id):
            lines[i:i + 1] = operation(line)
            return '\n'.join(lines) + '\n'
    raise AssertionError('test record not found')


class YarisWall(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.wall_bytes = (FIXTURE / 'wall.key').read_bytes()
        cls.combine_bytes = (FIXTURE / 'combine.key').read_bytes()
        cls.text = cls.wall_bytes.decode('ascii')
        cls.obj, cls.manifest = wall.compile_wall(cls.wall_bytes, cls.combine_bytes)
        cls.positions = {v['source_node_id']: v['position_m'] for v in cls.manifest['vertices']}

    def test_pinned_inputs_counts_ids_and_units(self):
        self.assertEqual(wall.sha256(self.wall_bytes), wall.WALL_SHA256)
        self.assertEqual(wall.sha256(self.combine_bytes), wall.COMBINE_SHA256)
        self.assertEqual(len(self.manifest['vertices']), 62)
        self.assertEqual(len(self.manifest['source_quads']), 46)
        self.assertEqual(len(self.manifest['triangles']), 100)
        self.assertEqual(set(self.positions), set(range(1001, 1063)))
        for v in self.manifest['vertices']:
            self.assertEqual(v['assembled_source_node_id'], v['source_node_id'] + 10000000)
            self.assertEqual(v['position_m'][0], (4600 - 4550) / 1000)
        for actual, expected in [(self.positions[1001], [0.05, -1.0540999756, 0.06682499695]),
                                 (self.positions[1062], [0.05, 1.05225, 1.759])]:
            for a, e in zip(actual, expected):
                self.assertAlmostEqual(a, e, delta=1e-14)
        self.assertEqual(self.manifest['transform']['translation_mm'], [-4550, 0, 0])
        self.assertEqual(self.manifest['output_length_unit'], 'm')

    def test_planarity_winding_positive_area_and_no_duplicate_vertices(self):
        self.assertEqual(len({tuple(p) for p in self.positions.values()}), 62)
        self.assertEqual({p[0] for p in self.positions.values()}, {0.05})
        for tri in self.manifest['triangles']:
            p, q, r = [self.positions[n] for n in tri['source_node_ids']]
            # Independent vector cross product, not importer geometry helpers.
            u, v = [[b[i] - p[i] for i in range(3)] for b in (q, r)]
            normal = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                      u[0] * v[1] - u[1] * v[0]]
            self.assertLess(normal[0], 0)
            self.assertEqual(normal[1:], [0, 0])

    def test_independent_shoelace_area_by_source_face_and_outer_boundary(self):
        # Decimal shoelace uses polygon edges, independently of fan selection.
        def area(ids):
            points = [(Decimal(str(self.positions[n][1])), Decimal(str(self.positions[n][2])))
                      for n in ids]
            return abs(sum(a[0] * b[1] - a[1] * b[0]
                           for a, b in zip(points, points[1:] + points[:1]))) / 2
        by_face = defaultdict(list)
        for tri in self.manifest['triangles']:
            by_face[tri['source_quad_id']].append(tri['source_node_ids'])
        for quad in self.manifest['source_quads']:
            self.assertAlmostEqual(float(sum(area(t) for t in by_face[quad['source_quad_id']])),
                                   float(area(quad['source_node_ids'])), places=13)
        # Traverse the independently known external wall outline. Intermediate
        # bottom/left collinear nodes do not alter area; the tapered right side
        # retains every source boundary kink.
        outline = [1001, 1055, 1056, 1057, 1058, 1059, 1060, 1062, 1061,
                   1006, 1005, 1004, 1003, 1002]
        actual = sum(area(t['source_node_ids']) for t in self.manifest['triangles'])
        self.assertAlmostEqual(float(actual), float(area(outline)), places=13)
        self.assertAlmostEqual(float(actual), 3.565455298182685, places=13)

    def test_shared_edges_conforming_seam_and_single_disk(self):
        edges = defaultdict(list)
        for t in self.manifest['triangles']:
            ids = t['source_node_ids']
            for i in range(3):
                a, b = ids[i], ids[(i + 1) % 3]
                edges[tuple(sorted((a, b)))].append((a, b, t['source_quad_id']))
        self.assertEqual(len(edges), 161)
        self.assertEqual(Counter(len(uses) for uses in edges.values()), {1: 22, 2: 139})
        self.assertEqual(62 - len(edges) + 100, 1)
        for uses in edges.values():
            if len(uses) == 2:
                self.assertEqual(uses[0][:2], uses[1][:2][::-1])
        chain = list(range(1006, 1061, 6))
        self.assertNotIn((1006, 1060), edges)
        for a, b in zip(chain, chain[1:]):
            uses = edges[(a, b)]
            self.assertEqual(len(uses), 2)
            self.assertEqual(sum(face == 1046 for _, _, face in uses), 1)
        self.assertEqual(self.manifest['stitching']['edges'][0]['inserted_source_node_ids'],
                         chain[1:-1])

    def test_source_coverage_and_reaction_sets(self):
        counts = Counter(t['source_quad_id'] for t in self.manifest['triangles'])
        self.assertEqual(set(counts), set(range(1001, 1047)))
        self.assertEqual(counts[1046], 10)
        self.assertTrue(all(counts[q] == 2 for q in range(1001, 1046)))
        regions = self.manifest['reaction_groups']
        whole = set(regions['whole_wall_triangle_ids'])
        lower = set(regions['source_segment_set_1001_triangle_ids'])
        self.assertEqual(whole, set(range(1, 101)))
        self.assertEqual(len(lower), 90)
        self.assertEqual(whole - lower,
                         {t['triangle_id'] for t in self.manifest['triangles'] if t['source_quad_id'] == 1046})
        for t in self.manifest['triangles']:
            self.assertEqual(t['assembled_source_quad_id'], t['source_quad_id'] + 10000000)
            self.assertEqual([self.manifest['vertices'][i]['source_node_id'] for i in t['vertex_indices']],
                             t['source_node_ids'])

    def test_actual_obj_mesh_matches_manifest(self):
        vertices, faces = [], []
        for line in self.obj.decode('ascii').splitlines():
            parts = line.split()
            if parts and parts[0] == 'v':
                vertices.append([float(x) for x in parts[1:]])
            elif parts and parts[0] == 'f':
                faces.append([int(x) - 1 for x in parts[1:]])
        self.assertEqual(vertices, [v['position_m'] for v in self.manifest['vertices']])
        self.assertEqual(faces, [t['vertex_indices'] for t in self.manifest['triangles']])
        self.assertEqual(wall.sha256(self.obj), self.manifest['artifacts']['wall.obj']['sha256'])
        self.assertEqual(wall.sha256(self.obj), '08cb67124534ec16fab19e9503472c339eee9f0c79cc4ca20d3d6b014d6ed4e9')

    def test_mesh_boundary_and_thickness_are_authoritative(self):
        settings = self.manifest['contact']
        self.assertFalse(settings['analytic_force_generation'])
        self.assertEqual(settings['dynamic_wall_dofs'], 0)
        self.assertEqual(settings['velocity_m_per_s'], [0, 0, 0])
        self.assertEqual(settings['additional_wall_offset_m'], 0)
        self.assertEqual(settings['source_display_thickness_m'], 0.001)
        self.assertEqual(settings['friction'], 0.6)
        upper = settings['analytic_source_regions'][1]['source_analytic_bounds_m']
        self.assertAlmostEqual(upper[0][2] - self.positions[1060][2], 0.001)
        self.assertAlmostEqual(upper[1][2] - self.positions[1062][2], 0.001)

    def test_missing_duplicate_or_invalid_records_fail(self):
        cases = [
            change_record(self.text, '*NODE', 1001, lambda _: []),
            change_record(self.text, '*NODE', 1001, lambda r: [r, r]),
            change_record(self.text, '*ELEMENT_SHELL', 1001, lambda _: []),
            change_record(self.text, '*ELEMENT_SHELL', 1001, lambda r: [r, r]),
            change_record(self.text, '*NODE', 1001, lambda r: [r[:8] + 'not_a_number'.rjust(16) + r[24:]]),
            change_record(self.text, '*NODE', 1001, lambda r: [r[:8] + 'NaN'.rjust(16) + r[24:]]),
            change_record(self.text, '*NODE', 1001, lambda r: [r[:8] + '4601.0'.rjust(16) + r[24:]]),
            change_record(self.text, '*ELEMENT_SHELL', 1001, lambda r: [r[:16] + '9999'.rjust(8) + r[24:]]),
            change_record(self.text, '*ELEMENT_SHELL', 1001, lambda r: [r[:16] + r[24:32] + r[24:]]),
            self.text.replace('*NODE', '*NODE_SCALAR'),
            self.text.replace('*END', ''),
        ]
        for i, text in enumerate(cases):
            with self.subTest(case=i), self.assertRaises(wall.WallImportError):
                wall.parse_wall(text)

    def test_nonconforming_source_edits_and_reversed_quad_fail(self):
        parsed = wall.parse_wall(self.text)
        ids = parsed['quads'][1046]
        parsed['quads'][1046] = tuple(reversed(ids))
        with self.assertRaises(wall.WallImportError):
            wall.triangulate(parsed)
        parsed = wall.parse_wall(self.text)
        x, y, z = parsed['nodes'][1012]
        parsed['nodes'][1012] = (x, y, z + 0.01)
        with self.assertRaises(wall.WallImportError):
            wall.triangulate(parsed)

    def test_transform_and_pin_failures(self):
        text = self.combine_bytes.decode('ascii')
        for modified in [text.replace('TRANSL', 'ROTATE'), text.replace('-4550.0', '-4500.0'),
                         text.replace('wall.key', 'missing.key'), text.replace('*END', '')]:
            with self.subTest(modified=modified[:20]), self.assertRaises(wall.WallImportError):
                wall.parse_transform(modified)
        with self.assertRaisesRegex(wall.WallImportError, 'SHA256'):
            wall.compile_wall(self.wall_bytes + b'\n', self.combine_bytes)
        with self.assertRaisesRegex(wall.WallImportError, 'SHA256'):
            wall.compile_wall(self.wall_bytes, self.combine_bytes + b'\n')

    def test_repeatable_output_and_all_manifest_hashes(self):
        with tempfile.TemporaryDirectory(prefix='yaris-wall-') as tmp:
            out = Path(tmp)
            wall.write_artifacts(out, self.obj, self.manifest, self.wall_bytes, self.combine_bytes)
            first = {p.relative_to(out): p.read_bytes() for p in out.rglob('*') if p.is_file()}
            wall.write_artifacts(out, self.obj, self.manifest, self.wall_bytes, self.combine_bytes)
            second = {p.relative_to(out): p.read_bytes() for p in out.rglob('*') if p.is_file()}
            self.assertEqual(first, second)
            for line in (out / 'SHA256SUMS').read_text().splitlines():
                checksum, name = line.split('  ', 1)
                self.assertEqual(checksum, wall.sha256((out / name).read_bytes()))
            self.assertEqual((out / 'source/wall.key').read_bytes(), self.wall_bytes)
            self.assertEqual(json.loads((out / 'manifest.json').read_text()), self.manifest)

    def test_missing_cli_inputs_publish_no_mesh(self):
        with tempfile.TemporaryDirectory(prefix='yaris-wall-missing-') as tmp:
            output = Path(tmp) / 'output'
            with patch.object(sys, 'argv', [str(SCRIPT), '--source-model', str(Path(tmp) / 'missing'),
                                           '--output', str(output)]):
                with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                    wall.main()
            self.assertEqual(error.exception.code, 1)
            self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()
