from io import BytesIO
import unittest
import math
from test_canonical_part_geometry import row
from modelio.source_blocks import scan_declarations
from modelio.declarations import UnitSystem
from modelio.keyword_cards import parse_material
from modelio.law44_declarations import parse_linear_law44_material
from modelio.constant_failure_declarations import parse_constant_failure_material


class ConstantFailureDeclarations(unittest.TestCase):
    units = UnitSystem('t', 'mm', 's', 1000, .001, 1)

    def block(self, fail=1, curve=47, tangent=None, **changes):
        values = dict(mid=37, rho='7.8900E-9', young='2.0000E+5', nu=.3,
                      sigy=270, tangent=tangent, fail=fail, tdel=None)
        values.update(changes)
        raw = ('*KEYWORD\n*MAT_PIECEWISE_LINEAR_PLASTICITY\n' + row(*values.values()) + '\n' +
               row(8000, 8, curve, None, 0) + '\n' + row() + '\n' + row() + '\n*END\n').encode()
        return scan_declarations(BytesIO(raw), 'yaris-coarse-v1l.key').one('material', 37)

    def test_three_original_failure_values_preserve_cards_and_units(self):
        for fail, curve, tangent in ((1, 47, None), (1, 47, 0.), (1, 47, -0.), (.25, 0, 800), (3.5, 0, 900)):
            block = self.block(fail, curve, tangent)
            parsed = parse_constant_failure_material(block, self.units)
            self.assertEqual(parsed.failure_strain, fail)
            self.assertEqual(parsed.material.source, block)
            self.assertEqual(parsed.material.young_pa, 2e11)
            self.assertEqual(parsed.material.cards[0].get('fail'), fail)
            self.assertEqual(parsed.material.hardening_curve_id, curve)
            if tangent is not None:
                self.assertEqual(math.copysign(1, parsed.material.supplied_etan_pa), math.copysign(1, tangent))
            with self.assertRaises(ValueError): parse_material(block, self.units)
            with self.assertRaises(ValueError): parse_linear_law44_material(block, self.units)

    def test_invalid_fail_tdel_and_time_do_not_normalize(self):
        for fail in (None, 0, -0., -1):
            with self.assertRaises(ValueError):
                parse_constant_failure_material(self.block(fail), self.units)
        with self.assertRaises(ValueError):
            parse_constant_failure_material(self.block(tdel=0), self.units)
        with self.assertRaises(ValueError):
            parse_constant_failure_material(self.block(tangent=1), self.units)
        with self.assertRaises(ValueError):
            parse_constant_failure_material(self.block(), UnitSystem('t', 'mm', 'ms', 1000, .001, .001))
        self.assertEqual(parse_constant_failure_material(self.block(), self.units).failure_strain, 1)


if __name__ == '__main__': unittest.main()
