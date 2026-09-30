"""Small source declaration/coordinate checks; full original fixture is root gated."""
from dataclasses import FrozenInstanceError
from io import BytesIO
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from modelio.source_blocks import scan_declarations
from modelio.type13_declarations import parse_beam_property
from modelio.type13_coordinates import collect_working_coordinates


def row(*values, width=10):
    return ''.join(('' if v is None else str(v)).rjust(width) for v in values).rstrip()


def declaration(*, inner=None, poisson=.3, failure=None):
    text = '\n'.join(['*KEYWORD', '*PART', 'original beam', row(2000486, 2000486, 2000486),
        '*SECTION_BEAM', row(2000486, 9, 1., 0, 1), row(5., 5., inner),
        '*MAT_SPOTWELD', row(2000486, '7.8e-9', 50000., poisson, 300., 5000., None, '1e20'),
        row('2e20', failure), '*END', ''])
    index = scan_declarations(BytesIO(text.encode()), 'source.key')
    return parse_beam_property(index.one('part', 2000486), index.one('section', 2000486), index.one('material', 2000486))


def coordinate_fixture():
    beam = row(100, 2000486, 1, 2, 3, None, None, None, None, 2, width=8)
    def node(nid, xyz):
        return str(nid).rjust(8)+''.join(('' if x is None else str(x)).rjust(16) for x in xyz)
    text = '\n'.join(['*KEYWORD', '*ELEMENT_BEAM', beam, '*NODE', node(1, (100, 20, 30)),
                      node(2, (101, 22, 33)), node(3, (None, None, None)), '*END', ''])
    nodes = {1: dict(source_line=5, blank_mask=48, position_m=(.1, .02, .03), codes=(0, 0)),
             2: dict(source_line=6, blank_mask=48, position_m=(.101, .022, .033), codes=(0, 0)),
             3: dict(source_line=7, blank_mask=62, position_m=(0., 0., 0.), codes=(0, 0))}
    beams = {100: dict(source_id=100, canonical_index=0, source_line=3,
                      raw_record=(100, 2000486, 1, 2, 3, 0, 0, 0, 0, 2), blank_mask=480)}
    return text, nodes, beams


class Type13Source(unittest.TestCase):
    def test_original_fields_preserved_without_python_conversion(self):
        result = declaration()
        self.assertEqual(result.section_cards[1].values[2:], (None,) * 6)
        self.assertEqual(result.material_cards[0].get('tfail'), 1e20)
        self.assertEqual(result.material_cards[1].get('efail'), 2e20)
        self.assertFalse(hasattr(result, 'native_curves'))
        with self.assertRaises(FrozenInstanceError):
            result.section_source = None

    def test_supplied_optional_fields_and_missing_poisson_reject(self):
        for options in [dict(inner=0), dict(poisson=None), dict(poisson=.5), dict(failure=0)]:
            with self.subTest(options=options), self.assertRaises(ValueError):
                declaration(**options)
        self.assertEqual(declaration().part.part_id, 2000486)

    def test_original_coordinates_and_n3_blanks_are_kept(self):
        text, nodes, beams = coordinate_fixture()
        result, rows, summary = collect_working_coordinates(BytesIO(text.encode()), nodes, beams, 2000486)
        self.assertEqual(result[1].position_native, (100., 20., 30.))
        self.assertEqual(result[3].position_native, (0., 0., 0.))
        self.assertEqual(result[3].blank_mask, 62)
        self.assertEqual(rows[100]['raw_record'][4], 3)
        self.assertEqual(summary['keyword_counts']['*NODE'], 1)

    def test_late_coordinate_mismatch_duplicate_and_missing_coverage_reject_then_retry(self):
        text, nodes, beams = coordinate_fixture()
        nodes[3] = dict(nodes[3], position_m=(1., 0., 0.))
        with self.assertRaises(ValueError):
            collect_working_coordinates(BytesIO(text.encode()), nodes, beams, 2000486)
        text, nodes, beams = coordinate_fixture()
        duplicate = text.replace('*END', text.splitlines()[6]+'\n*END')
        with self.assertRaises(ValueError):
            collect_working_coordinates(BytesIO(duplicate.encode()), nodes, beams, 2000486)
        missing = text.replace(text.splitlines()[6]+'\n', '')
        with self.assertRaises(ValueError):
            collect_working_coordinates(BytesIO(missing.encode()), nodes, beams, 2000486)
        self.assertEqual(len(collect_working_coordinates(BytesIO(text.encode()), nodes, beams, 2000486)[0]), 3)


if __name__ == '__main__':
    unittest.main()
