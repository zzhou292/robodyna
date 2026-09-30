"""Analytic source declarations, independent of a solver or a vehicle run."""
from dataclasses import FrozenInstanceError
from io import BytesIO
from pathlib import Path
import math
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from modelio.declarations import UnitSystem
from modelio.keyword_cards import parse_material
from modelio.law44_declarations import (parse_linear_law44_material, linear_law44_candidates,
                                       LINEAR_LAW44_POLICY)
from modelio.source_blocks import scan_declarations
from test_source_part_declarations import row

SOURCE_UNITS = UnitSystem('t', 'mm', 's', 1000, .001, 1)
SI_UNITS = UnitSystem('kg', 'm', 's', 1, 1, 1)


def material_block(first=None, second=None, eps='', stress='', newline='\n'):
    first = first if first is not None else [37, '1.0000E-9', 1000, .3, 20, 10]
    second = second if second is not None else [8000, 8, 0, None, '0.0']
    raw = newline.join(['*KEYWORD', '*MAT_PIECEWISE_LINEAR_PLASTICITY',
                        row(*first), row(*second), eps, stress, '*END', '']).encode('ascii')
    return scan_declarations(BytesIO(raw), 'analytic.key').one('material', 37)


class LinearLaw44Declarations(unittest.TestCase):
    def test_original_scalar_tuples_use_native_modulus_and_keep_source_cards(self):
        for young, sigy, etan in [(1000, 20, 10), (3000, 60, 30), (6000, 60, 60)]:
            block = material_block([37, '1.0000E-9', young, .3, sigy, etan], newline='\r\n')
            parsed = parse_linear_law44_material(block, SOURCE_UNITS)
            material = parsed.material
            self.assertIs(material.source, block)
            self.assertEqual(material.young_pa, young * 1e6)
            self.assertEqual(material.supplied_sigy_pa, sigy * 1e6)
            self.assertEqual(material.supplied_etan_pa, etan * 1e6)
            self.assertEqual(material.hardening_curve_id, 0)
            self.assertEqual(material.rate_coefficient_per_s, 8000)
            self.assertEqual(material.rate_exponent, 8)
            self.assertEqual(material.rate_type, 0)
            expected = (etan * 1e6) * (young * 1e6) / ((young - etan) * 1e6)
            self.assertEqual(struct.pack('<d', parsed.plastic_hardening_pa), struct.pack('<d', expected))
            for card in material.cards[2:]:
                self.assertEqual(card.blank_field_mask, 255)
                self.assertEqual(card.values, (None,) * 8)
            self.assertIn('\r\n', material.source.raw_text)
            with self.assertRaises(FrozenInstanceError):
                parsed.plastic_hardening_pa = 0
        self.assertFalse(LINEAR_LAW44_POLICY['case_integration_qualified'])

    def test_table_and_analytic_parsers_retain_separate_explicit_domains(self):
        analytic = material_block()
        with self.assertRaises(ValueError):
            parse_material(analytic, SOURCE_UNITS)
        table = material_block(second=[8000, 8, 47, None, 0])
        old = parse_material(table, SOURCE_UNITS)
        self.assertEqual(old.hardening_curve_id, 47)
        self.assertEqual(old.supplied_sigy_pa, 20e6)
        with self.assertRaises(ValueError):
            parse_linear_law44_material(table, SOURCE_UNITS)
        self.assertEqual(parse_material(table, SOURCE_UNITS), old)

    def test_unresolved_rate_failure_inline_and_optional_fields_are_not_repaired(self):
        mutations = []
        for index, value in [(0, None), (0, 0), (1, None), (1, 0), (2, None),
                             (2, -1), (2, 47), (3, 0), (4, None), (4, 1), (5, 0)]:
            second = [8000, 8, 0, None, 0, None, None, None]
            second[index] = value
            mutations.append(dict(second=second))
        for index, value in [(3, -.1), (4, None), (4, 0), (4, -1), (5, None),
                             (5, -1), (5, 1000), (5, 2000), (6, .25), (7, 0)]:
            first = [37, '1.0000E-9', 1000, .3, 20, 10, None, None]
            first[index] = value
            mutations.append(dict(first=first))
        mutations.extend([dict(eps=row(0)), dict(stress=row(20))])
        for values in mutations:
            with self.subTest(values=values), self.assertRaises(ValueError):
                parse_linear_law44_material(material_block(**values), SOURCE_UNITS)

    def test_finite_source_modulus_overflow_and_underflow_reject_then_retry(self):
        for young, etan in [('1e200', '5e199'), ('1e-200', '1e-300')]:
            with self.subTest(young=young), self.assertRaisesRegex(ValueError, 'modulus'):
                parse_linear_law44_material(material_block([37, 1, young, .3, 1, etan]), SI_UNITS)
        good = parse_linear_law44_material(material_block(), SOURCE_UNITS)
        self.assertTrue(math.isfinite(good.plastic_hardening_pa))
        perfect = parse_linear_law44_material(material_block([37, '1e-9', 1000, .3, 20, '-0.0']), SOURCE_UNITS)
        self.assertEqual(perfect.plastic_hardening_pa, 0)
        self.assertEqual(struct.pack('<d', perfect.material.supplied_etan_pa), struct.pack('<d', -0.0))
        with self.assertRaisesRegex(ValueError, 'source seconds'):
            parse_linear_law44_material(material_block(), UnitSystem('kg', 'm', 'ms', 1, 1, .001))

    def test_material_candidates_preserve_separate_part_section_and_simulation_gates(self):
        from types import SimpleNamespace
        block = material_block()
        index = SimpleNamespace(one=lambda family, identity: block)
        parts = [dict(retained_shell_part=True, source_part_id=1, source_material_id=37,
                      counts=dict(shells=9), existing_adapter_rejections=[]),
                 dict(retained_shell_part=True, source_part_id=2, source_material_id=37,
                      counts=dict(shells=7), existing_adapter_rejections=[dict(family='section', reason='NIP1')])]
        report = linear_law44_candidates(index, parts, SOURCE_UNITS)
        self.assertEqual(report['shells'], 16)
        self.assertEqual(report['shells_with_supported_part_and_section'], 9)
        self.assertEqual(report['parts'][0]['source_material_sha256'], block.sha256)
        self.assertFalse(report['simulation_ready'])
        self.assertFalse(report['policy']['case_integration_qualified'])
        parts[0]['retained_shell_part'] = False
        self.assertEqual(linear_law44_candidates(index, parts, SOURCE_UNITS)['shells'], 7)


if __name__ == '__main__':
    unittest.main()
