"""Strict original elastic source semantics, independent of resident admission."""
from dataclasses import FrozenInstanceError
from io import BytesIO
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from modelio.declarations import UnitSystem
from modelio.law1_declarations import parse_elastic_material, layered_law1_candidates
from modelio.source_blocks import scan_declarations
from test_source_part_declarations import row

UNITS = UnitSystem('t', 'mm', 's', 1000, .001, 1)


def block(values=None, extra='', newline='\n'):
    values = [37, '7.8900E-9', '2.0000E+5', .3] if values is None else values
    text = newline.join(['*KEYWORD', '*MAT_ELASTIC', row(*values), extra, '*END', ''])
    # A blank extra row would itself be a source card, so omit it in valid input.
    if not extra:
        text = newline.join(['*KEYWORD', '*MAT_ELASTIC', row(*values), '*END', ''])
    return scan_declarations(BytesIO(text.encode('ascii')), 'elastic.key').one('material', 37)


class ElasticDeclarations(unittest.TestCase):
    def test_original_coefficients_and_blank_source_fields_are_owned_without_plastic_state(self):
        for density, young in [('7.8900E-9', '2.0000E+5'), ('7.890E-10', '10000.000')]:
            source = block([37, density, young, .3], newline='\r\n')
            parsed = parse_elastic_material(source, UNITS)
            self.assertIs(parsed.source, source)
            self.assertEqual(struct.pack('<d', parsed.density_kg_m3), struct.pack('<d', float(density) * UNITS.density_to_si))
            self.assertEqual(parsed.young_pa, float(young) * 1e6)
            self.assertEqual(parsed.cards[0].blank_field_mask, 240)
            self.assertEqual(parsed.cards[0].values[4:], (None,) * 4)
            self.assertIn('\r\n', parsed.source.raw_text)
            self.assertFalse(hasattr(parsed, 'supplied_sigy_pa'))
            with self.assertRaises(FrozenInstanceError):
                parsed.young_pa = 0

    def test_missing_nonfinite_or_supplied_optional_fields_reject(self):
        for index, value in [(1, None), (1, 0), (1, -1), (2, None), (2, 0),
                             (2, '1e308'), (3, None), (3, -1), (3, -.1), (3, .5),
                             (4, 0), (5, 0), (6, 0), (7, 0)]:
            values = [37, '7.89e-9', '2e5', .3, None, None, None, None]
            values[index] = value
            with self.subTest(index=index, value=value), self.assertRaises(ValueError):
                parse_elastic_material(block(values), UNITS)
        with self.assertRaises(ValueError):
            parse_elastic_material(block(extra=row(0)), UNITS)
        parsed = parse_elastic_material(block([37, '7.89e-9', '2e5', '-0.0']), UNITS)
        self.assertEqual(struct.pack('<d', parsed.poisson_ratio), struct.pack('<d', -0.0))

    def test_source_si_overflow_and_underflow_do_not_change_a_later_valid_read(self):
        tiny = UnitSystem('kg', 'm', 's', 1e-200, 1, 1)
        for values, units in [([37, '1e308', 1, .3], UNITS), ([37, '1e-200', 1, .3], tiny),
                              ([37, 1, '1e-200', .3], tiny)]:
            with self.subTest(values=values), self.assertRaises(ValueError):
                parse_elastic_material(block(values), units)
        self.assertEqual(parse_elastic_material(block(), UNITS).young_pa, 2e11)

    def test_candidate_scope_preserves_exclusions_and_unresolved_section_roles(self):
        source = block()
        index = SimpleNamespace(one=lambda family, identity: source)
        parts = [dict(retained_shell_part=True, source_part_id=1, source_material_id=37,
                      counts=dict(shells=9), existing_adapter_rejections=[]),
                 dict(retained_shell_part=True, source_part_id=2, source_material_id=37,
                      counts=dict(shells=7), existing_adapter_rejections=[dict(family='section', reason='NIP1')]),
                 dict(retained_shell_part=False, source_part_id=3, source_material_id=37,
                      counts=dict(shells=100), existing_adapter_rejections=[])]
        report = layered_law1_candidates(index, parts, UNITS)
        self.assertEqual(report['shells'], 16)
        self.assertEqual(report['shells_with_supported_part_and_section'], 9)
        self.assertEqual(report['parts'][0]['source_material_sha256'], source.sha256)
        self.assertFalse(report['simulation_ready'])
        self.assertFalse(report['policy']['case_integration_qualified'])


if __name__ == '__main__':
    unittest.main()
