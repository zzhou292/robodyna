"""The original importer cannot hide tetra10 or multiline solid fields."""
import unittest
from modelio._legacy import VehicleGeometry, WallImportError


class TiedClassificationSourceTopology(unittest.TestCase):
    def test_original_solid_width_and_complete_row_are_required_before_append(self):
        def card(values):
            return ''.join(f'{value:8d}' for value in values)

        geometry = VehicleGeometry()
        geometry.record('*ELEMENT_SOLID', card([10, 20, 1, 2, 3, 4, 5, 6, 7, 8]), 1)
        original = geometry.elements['solids'].tobytes()
        for row in [card([11, 20, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]),
                    card([11, 20]), card([1, 2, 3, 4, 5, 6, 7, 8])]:
            with self.subTest(row=row):
                with self.assertRaises(WallImportError):
                    geometry.record('*ELEMENT_SOLID', row, 2)
                self.assertEqual(geometry.elements['solids'].tobytes(), original)
                self.assertEqual(geometry.element_ids, {10})
        geometry.record('*ELEMENT_SOLID', card([11, 20, 1, 2, 3, 4, 5, 6, 7, 8]), 2)
        self.assertEqual(geometry.element_ids, {10, 11})


if __name__ == '__main__':
    unittest.main()
