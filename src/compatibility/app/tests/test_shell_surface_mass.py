"""Independent lamina/moment oracles, using separately qualified donor rules.

Run with --quadrature PATH (or ROBO_DYNA_SURFACE_QUADRATURE). A missing donor
artifact is a failure, never a fabricated rule or skipped numerical test.
"""
import argparse
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from modelio.canonical_geometry import PartGeometry, SourceNode, SourceShell
from modelio.shell_mass import audit_shell_surface, load_quadrature
from modelio._legacy import file_sha256
from modelio import yaris_part
from test_canonical_part_geometry import TinyCanonical

QUADRATURE_PATH = os.environ.get('ROBO_DYNA_SURFACE_QUADRATURE')


def geometry(points, faces):
    nodes = tuple(SourceNode(i + 1, i, tuple(p), i + 1, (0, 0), 0) for i, p in enumerate(points))
    shells = []
    for i, face in enumerate(faces):
        raw = tuple(face) if len(face) == 4 else tuple(face) + (face[-1],)
        shells.append(SourceShell(101 + i, i, i + 10, (101 + i, 17) + tuple(n + 1 for n in raw),
                                  raw, raw, 0, len(face)))
    return PartGeometry(17, 27, 37, 'synthetic independent geometry', nodes, tuple(shells), '', '', '', '', ())


def saddle(a=.7, b=.4, k=.6):
    return [(x, y, k * x * y) for x, y in ((-a, -b), (a, -b), (a, b), (-a, b))]


def exact_saddle_area(a, b, k):
    # Independent closed rectangular integral of sqrt(1+k²(x²+y²)). The
    # divergence identity 3*integral(R)=boundary(xR,yR)+integral(1/R)
    # reduces it to asinh/atan endpoint expressions; no numerical quadrature.
    x, y = abs(k) * a, abs(k) * b
    r = math.sqrt(1 + x * x + y * y)
    ax = math.asinh(y / math.sqrt(1 + x * x))
    ay = math.asinh(x / math.sqrt(1 + y * y))
    inverse_integral = x * ax + y * ay - math.atan(x * y / r)
    boundary = .5 * (x * (y * r + (1 + x * x) * ax) + y * (x * r + (1 + y * y) * ay))
    return 4 * (boundary + inverse_integral) / (3 * k * k)


def rotation():
    axis = (2 / 3, -1 / 3, 2 / 3)
    angle = .83
    c, s = math.cos(angle), math.sin(angle)
    skew = ((0, -axis[2], axis[1]), (axis[2], 0, -axis[0]), (-axis[1], axis[0], 0))
    return tuple(tuple((c if i == j else 0) + (1 - c) * axis[i] * axis[j] + s * skew[i][j]
                       for j in range(3)) for i in range(3))


class ShellSurfaceMass(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not QUADRATURE_PATH:
            raise ValueError('provide the actual qualified quadrature artifact; no numerical tests are skipped')
        cls.quadrature = load_quadrature(QUADRATURE_PATH)
        with tempfile.TemporaryDirectory() as directory:
            d = TinyCanonical(directory).declarations
        # A unit areal-density lamina makes closed-form physical oracles direct.
        cls.declarations = replace(d, section=replace(d.section, thickness_m=(.5,) * 4),
                                   material=replace(d.material, density_kg_m3=2.))

    def audit(self, points, faces=((0, 1, 2, 3),), **kwargs):
        return audit_shell_surface(geometry(points, faces), self.declarations, self.quadrature, **kwargs)

    def close(self, actual, expected, tolerance=2e-11):
        self.assertLessEqual(abs(actual - expected), tolerance * max(1., abs(expected)))

    def test_rectangle_exact_area_com_and_full_inertia(self):
        result = self.audit([(-1, -.5, 0), (1, -.5, 0), (1, .5, 0), (-1, .5, 0)])
        for order in ('4', '8', '16'):
            r = result['orders'][order]
            self.close(r['area_m2'], 2.)
            self.close(r['mass_kg'], 2.)
            for x in r['centroid_m']:
                self.close(x, 0.)
            expected = (1 / 6, 0, 0, 0, 2 / 3, 0, 0, 0, 5 / 6)
            for actual, value in zip(r['central_inertia_kg_m2'], expected):
                self.close(actual, value)
        self.assertFalse(result['source_mass_equivalence_qualified'])
        self.assertFalse(result['simulation_ready'])

    def test_native_triangle_exact_nondiagonal_moments(self):
        result = self.audit([(0, 0, 0), (2, 0, 0), (0, 3, 0)], ((0, 1, 2),))
        r = result['orders']['16']
        self.close(r['mass_kg'], 3.)
        for actual, expected in zip(r['centroid_m'], (2 / 3, 1, 0)):
            self.close(actual, expected)
        for actual, expected in zip(r['central_inertia_kg_m2'], (1.5, .5, 0, .5, 2 / 3, 0, 0, 0, 13 / 6)):
            self.close(actual, expected)
        self.assertEqual(result['elements'][0]['arity'], 3)

    def test_mixed_q4_t3_coverage_matches_analytic_trapezoid(self):
        result = self.audit([(0, 0, 0), (2, 0, 0), (2, 1, 0), (0, 1, 0), (3, 0, 0)],
                            ((0, 1, 2, 3), (1, 4, 2)))
        r = result['orders']['16']
        self.close(r['mass_kg'], 2.5)
        for actual, expected in zip(r['centroid_m'], (19 / 15, 7 / 15, 0)):
            self.close(actual, expected)
        expected = (37 / 180, 37 / 360, 0, 37 / 360, 253 / 180, 0, 0, 0, 29 / 18)
        for actual, value in zip(r['central_inertia_kg_m2'], expected):
            self.close(actual, value)
        self.assertEqual([e['source_element_id'] for e in result['elements']], [101, 102])
        self.assertEqual([e['arity'] for e in result['elements']], [4, 3])

    def test_saddle_area_uses_bilinear_surface_not_either_diagonal(self):
        result = self.audit(saddle())
        r = result['orders']['16']
        self.close(r['area_m2'], exact_saddle_area(.7, .4, .6), 2e-10)
        for x in r['centroid_m']:
            self.close(x, 0.)
        for i in (1, 2, 3, 5, 6, 7):
            self.close(r['central_inertia_kg_m2'][i], 0.)
        diagnostic = result['elements'][0]
        for name in ('diagonal02_diagnostics', 'diagonal13_diagnostics'):
            self.assertGreater(abs(diagnostic[name]['split_area_m2'] - r['area_m2']), 1e-3)
        self.assertGreater(diagnostic['scaled_jacobian_lower_bound'], 0)

    def test_rotated_nonplanar_geometry_and_full_tensor_covariance(self):
        p = saddle()
        before = self.audit(p)['orders']['16']
        r, translation = rotation(), (.31, -.42, 2.3)
        moved = [tuple(sum(r[i][j] * v[j] for j in range(3)) + translation[i] for i in range(3)) for v in p]
        after = self.audit(moved)['orders']['16']
        self.close(before['mass_kg'], after['mass_kg'])
        for i in range(3):
            self.close(after['centroid_m'][i], translation[i])
            for j in range(3):
                expected = sum(r[i][a] * before['central_inertia_kg_m2'][3 * a + b] * r[j][b]
                               for a in range(3) for b in range(3))
                self.close(after['central_inertia_kg_m2'][3 * i + j], expected)

    def test_length_thickness_density_scaling(self):
        p = saddle()
        original = self.audit(p)['orders']['16']
        d = replace(self.declarations, section=replace(self.declarations.section, thickness_m=(1.,) * 4),
                    material=replace(self.declarations.material, density_kg_m3=10.))
        result = audit_shell_surface(geometry([tuple(3 * x for x in v) for v in p], ((0, 1, 2, 3),)), d,
                                     self.quadrature)['orders']['16']
        self.close(result['area_m2'], 9 * original['area_m2'])
        self.close(result['mass_kg'], 90 * original['mass_kg'])
        for a, b in zip(result['central_inertia_kg_m2'], original['central_inertia_kg_m2']):
            self.close(a, 810 * b)

    def test_large_common_offset_keeps_small_central_inertia(self):
        p = [(-1, -.5, 0), (1, -.5, 0), (1, .5, 0), (-1, .5, 0)]
        first = self.audit(p)['orders']['16']
        offset = (1e8, -1e8, 1e8)
        moved = self.audit([tuple(x + y for x, y in zip(v, offset)) for v in p])['orders']['16']
        self.assertEqual(moved['centroid_m'], offset)
        for a, b in zip(first['central_inertia_kg_m2'], moved['central_inertia_kg_m2']):
            self.close(a, b)

    def test_remote_tiny_element_keeps_source_representable_local_shape(self):
        delta = math.ulp(1e8)
        large = [(-1e8, -1e8, 0), (-1e8 + 1, -1e8, 0),
                 (-1e8 + 1, -1e8 + 1, 0), (-1e8, -1e8 + 1, 0)]
        tiny = [(1e8, 1e8, 0), (1e8 + delta, 1e8, 0),
                (1e8 + delta, 1e8 + delta, 0), (1e8, 1e8 + delta, 0)]
        result = self.audit(large + tiny, ((0, 1, 2, 3), (4, 5, 6, 7)))
        element = result['elements'][1]['orders']['16']
        self.assertLessEqual(abs(element['area_m2'] - delta**2), 1e-10 * delta**2)
        expected = delta**4 / 12
        self.assertLessEqual(abs(element['central_inertia_kg_m2'][0] - expected), 1e-10 * expected)

    def test_corner_norms_do_not_admit_interior_zero_or_concave_map(self):
        # Bow-tie corner Jacobian norms are positive, but its center is zero.
        for p in ([(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0)],
                  [(0, 0, 0), (2, 0, 0), (.2, .2, 0), (0, 2, 0)]):
            with self.assertRaisesRegex(ValueError, 'Jacobian'):
                self.audit(p)
        with self.assertRaisesRegex(ValueError, 'degenerate triangle'):
            self.audit([(0, 0, 0), (1, 0, 0), (2, 0, 0)], ((0, 1, 2),))

    def test_per_element_convergence_cannot_be_hidden_by_massive_planar_neighbor(self):
        small = [tuple(1e-4 * x for x in p) for p in saddle(1, 1, 8)]
        large = [(2, 2, 0), (10002, 2, 0), (10002, 10002, 0), (2, 10002, 0)]
        # The small patch's whole mass is <5e-7 versus the neighbor's 1e8;
        # aggregate relative mass error could hide even its complete omission.
        with self.assertRaisesRegex(ValueError, 'element 101.*did not converge'):
            self.audit(small + large, ((0, 1, 2, 3), (4, 5, 6, 7)))

    def test_near_collinear_map_rejected_after_rescaling_and_rotation(self):
        r = rotation()
        thin = [(0, 0, 0), (1, 0, 0), (1, 1e-15, 0), (0, 1e-15, 0)]
        for scale in (1e-12, 1., 1e12):
            for rotated in (False, True):
                p = [tuple(scale * (sum(r[i][j] * v[j] for j in range(3)) if rotated else v[i])
                           for i in range(3)) for v in thin]
                with self.subTest(scale=scale, rotated=rotated), self.assertRaisesRegex(ValueError, 'Jacobian'):
                    self.audit(p)

    def test_bad_density_thickness_overflow_and_retry(self):
        g = geometry(saddle(), ((0, 1, 2, 3),))
        before = audit_shell_surface(g, self.declarations, self.quadrature)
        for thickness in ((None, .5, .5, .5), (.5, .6, .5, .5), (0.,) * 4, (1e-308,) * 4):
            d = replace(self.declarations, section=replace(self.declarations.section, thickness_m=thickness))
            if thickness[0] == 1e-308:
                d = replace(d, material=replace(d.material, density_kg_m3=1e-308))
            with self.assertRaises(ValueError):
                audit_shell_surface(g, d, self.quadrature)
        with self.assertRaises(ValueError):
            self.audit([(0, 0, 0), (1e308, 0, 0), (1e308, 1e308, 0), (0, 1e308, 0)])
        with self.assertRaisesRegex(ValueError, 'tolerance'):
            self.audit(saddle(), refinement_tolerance=1e-6)
        with self.assertRaisesRegex(ValueError, 'did not converge'):
            self.audit(saddle(), refinement_tolerance=1e-30)
        self.assertEqual(before, audit_shell_surface(g, self.declarations, self.quadrature))

    def test_quadrature_claims_are_rechecked_and_bad_json_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'rule.json'
            source = json.loads(Path(QUADRATURE_PATH).read_text())
            source['rules'][0]['weights'][0] += 1e-4
            path.write_text(json.dumps(source))
            with self.assertRaises(ValueError):
                load_quadrature(path)
            path.write_text('{"schema":"x","schema":"x"}')
            with self.assertRaisesRegex(ValueError, 'duplicate JSON'):
                load_quadrature(path)
            path.write_text('{}')
            with self.assertRaisesRegex(ValueError, 'malformed'):
                load_quadrature(path)

    def test_direct_call_identity_and_triangle_slot_contradictions_fail(self):
        g = geometry([(0, 0, 0), (1, 0, 0), (0, 1, 0)], ((0, 1, 2),))
        duplicate = replace(g, shells=(g.shells[0], g.shells[0]))
        with self.assertRaisesRegex(ValueError, 'duplicate source element'):
            audit_shell_surface(duplicate, self.declarations, self.quadrature)
        malformed = replace(g.shells[0], local_node_indices=(0, 1, 2, 0), raw_record=(101, 17, 1, 2, 3, 1))
        with self.assertRaisesRegex(ValueError, 'connectivity'):
            audit_shell_surface(replace(g, shells=(malformed,)), self.declarations, self.quadrature)

    def test_end_to_end_composer_source_units_hashes_and_create_only_publication(self):
        with tempfile.TemporaryDirectory() as directory:
            f = TinyCanonical(directory)
            readme = ('Mass **t**: metric ton (1,000 kg)\nLength **mm**: millimeter\n'
                      'Force **N**: newton\nTime **s**: second\n')
            with zipfile.ZipFile(f.archive, 'w') as archive:
                archive.writestr('fixture/yaris-coarse-v1l.key', f.raw)
                archive.writestr('fixture/README.md', readme)
            f.reference.update(model='authored mixed-shell fixture', archive_sha256=file_sha256(f.archive))
            f.manifest['source_archive']['sha256'] = f.reference['archive_sha256']
            f.save()
            reference = Path(directory) / 'reference.json'
            reference.write_text(json.dumps(f.reference))
            destination = Path(directory) / 'readiness.json'
            with patch.object(yaris_part, 'REFERENCE', reference):
                result = yaris_part.compile_archive_readiness(f.archive, f.directory, QUADRATURE_PATH, 17)
                self.assertTrue(result['selected_geometry_source_verified'])
                self.assertFalse(result['simulation_ready'])
                self.assertFalse(result['source_mass_equivalence_qualified'])
                self.assertEqual([s['source_id'] for s in result['geometry']['shells']], [101, 102])
                self.assertEqual(result['geometry']['shells'][1]['raw_record'], (102, 17, 2, 5, 3, 3))
                self.assertEqual(result['geometry']['archive_sha256'], file_sha256(f.archive))
                self.assertEqual(result['geometry']['canonical_manifest_sha256'], file_sha256(f.directory / 'manifest.json'))
                for name, digest in result['geometry']['array_sha256']:
                    self.assertEqual(digest, file_sha256(f.directory / f.manifest['arrays'][name]['file']))
                declared = result['source_declarations']
                self.assertEqual(declared['source']['unit_authority']['raw_text'], readme)
                self.assertEqual(declared['declarations']['units']['mass_to_kg'], 1000.)
                self.assertEqual(declared['declarations']['units']['length_to_m'], .001)
                mass = result['surface_mass_audit']
                self.close(mass['orders']['16']['mass_kg'], 2.5 * 7890 * .001648)
                self.assertEqual(mass['quadrature']['artifact_sha256'], file_sha256(QUADRATURE_PATH))
                for name, digest in result['readiness_generator_sources'].items():
                    self.assertEqual(digest, file_sha256(Path(__file__).resolve().parents[1] / name))
                yaris_part.write_report(destination, result)
                before = destination.read_bytes()
                self.assertEqual(json.loads(before)['schema'], 'robo-dyna.source-part-readiness.v1')
                with self.assertRaises(FileExistsError):
                    yaris_part.write_report(destination, result)
                self.assertEqual(destination.read_bytes(), before)
                bad_rule = json.loads(Path(QUADRATURE_PATH).read_text())
                bad_rule['rules'][2]['nodes'][0] += 1e-5
                broken = Path(directory) / 'bad-rule.json'
                broken.write_text(json.dumps(bad_rule))
                rejected = Path(directory) / 'must-not-exist.json'
                with self.assertRaises(ValueError):
                    yaris_part.write_report(rejected, yaris_part.compile_archive_readiness(f.archive, f.directory, broken, 17))
                self.assertFalse(rejected.exists())
                self.assertEqual(destination.read_bytes(), before)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--quadrature')
    args, remaining = parser.parse_known_args()
    QUADRATURE_PATH = args.quadrature or QUADRATURE_PATH
    unittest.main(argv=[sys.argv[0]] + remaining)
