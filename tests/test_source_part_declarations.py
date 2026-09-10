"""Independent fixed-width source fixtures; no GPU, solver or original asset."""
from dataclasses import asdict
from io import BytesIO
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from modelio._legacy import WallImportError, sha256
from modelio.declarations import UnitSystem
from modelio.keyword_cards import compile_part_declarations
from modelio.source_blocks import scan_declarations
from modelio.yaris_part import declaration_report, write_report


def row(*values, width=10):
    return ''.join(' ' * width if value is None else str(value).rjust(width) for value in values)


def blocks():
    return {
        'part': ['*PART', '$ title comment', 'synthetic connector', row(17, 27, 37)],
        'section': ['*SECTION_SHELL', row(27, 2, None, 3), row(1.648, 1.648, 1.648, 1.648)],
        'material': ['*MAT_PIECEWISE_LINEAR_PLASTICITY',
                     row(37, '7.8900E-9', '2.0000E+5', .3, 270),
                     row(8000, 8, 47, None, '0.0'), '$ blank EPS card follows', '',
                     '$ blank ES card follows', ''],
        'curve': ['*DEFINE_CURVE', row(47, 0, 1, 1), row(0, 270, width=20),
                  row(.1, 340, width=20), row(.3, 362, width=20)],
    }


def deck(source=None, newline='\n', tail=()):
    source = blocks() if source is None else source
    lines = ['*KEYWORD']
    for block in source.values():
        lines.extend(block)
    return (newline.join(lines + list(tail) + ['*END', ''])).encode('ascii')


def compile_source(raw):
    index = scan_declarations(BytesIO(raw), 'synthetic.key')
    return compile_part_declarations(index, 17, UnitSystem('t', 'mm', 's', 1000, .001, 1))


class SourcePartDeclarations(unittest.TestCase):
    def test_explicit_si_units_curve_precedence_and_rate_flag(self):
        typed = compile_source(deck())
        self.assertAlmostEqual(typed.material.density_kg_m3, 7890)
        self.assertEqual(typed.material.young_pa, 200e9)
        self.assertAlmostEqual(typed.section.thickness_m[0], .001648)
        self.assertEqual(typed.material.supplied_sigy_pa, 270e6)
        self.assertIsNone(typed.material.supplied_etan_pa)
        self.assertEqual(typed.material.rate_coefficient_per_s, 8000)
        self.assertEqual(typed.material.rate_type, 0)
        self.assertFalse(typed.material.cards[1].blank_field_mask & (1 << 4))
        self.assertIn('0.0', typed.material.cards[1].raw_data_text)
        self.assertEqual(typed.hardening_curve.stress_pa, (270e6, 340e6, 362e6))
        report = declaration_report(typed)
        self.assertFalse(report['simulation_ready'])
        self.assertEqual(report['interpretation']['rate_type'], 'total_strain_rate')
        self.assertIn('ignored', report['interpretation']['supplied_SIGY_and_ETAN'])

    def test_blank_cards_line_numbers_and_exact_block_hash(self):
        raw = deck(newline='\r\n')
        typed = compile_source(raw)
        block = typed.material.source
        physical_lines = raw.splitlines(keepends=True)
        exact = b''.join(physical_lines[block.first_line - 1:block.last_line])
        self.assertEqual(block.raw_text.encode('ascii'), exact)
        self.assertEqual(block.sha256, sha256(exact))
        self.assertEqual(len(typed.material.cards), 4)
        for card in typed.material.cards[2:]:
            self.assertEqual(card.blank_field_mask, 255)
            self.assertEqual(card.values, (None,) * 8)
            self.assertEqual(physical_lines[card.source_line - 1], b'\r\n')
        self.assertIsNone(typed.section.cards[1].get('nloc'))
        self.assertIsNone(typed.section.cards[1].get('marea'))
        self.assertIsNone(typed.part.cards[0].get('hgid'))

    def test_curve_plus_rates_guidance_uses_pinned_converter_without_admission(self):
        typed = compile_source(deck())
        before = asdict(typed)
        report = declaration_report(typed)
        guidance = report['interpretation']
        self.assertIn('selects LAW44', guidance['donor_reference'])
        self.assertIn('earlier TABLE and LCSR branches select LAW36', guidance['donor_reference'])
        self.assertIn('B=0', guidance['donor_options'])
        self.assertIn('VP=0/ISMOOTH=1', guidance['donor_options'])
        self.assertIn('unqualified', guidance['donor_options'])
        self.assertIn('a62b27e6baa555d222a580d6218867d0be4d70b5',
                      report['interpretation_references']['urls']['pinned_material_converter'])
        self.assertFalse(report['simulation_ready'])
        self.assertEqual(asdict(typed), before)

    def test_mapping_guidance_does_not_admit_table_or_lcsr_branches(self):
        source = blocks()
        source['material'][2] = row(8000, 8, 47, 48, 0)
        with self.assertRaises(WallImportError):
            compile_source(deck(source))
        source = blocks()
        source['curve'][0] = '*DEFINE_TABLE'
        with self.assertRaises(WallImportError):
            compile_source(deck(source))

    def test_missing_blank_material_card_is_not_repaired(self):
        source = blocks()
        source['material'].pop()
        with self.assertRaisesRegex(WallImportError, 'exactly 4'):
            compile_source(deck(source))

    def test_blank_vp_remains_unresolved_and_curve_defaults_are_recorded(self):
        source = blocks()
        source['material'][2] = row(8000, 8, 47)
        source['curve'][1] = row(47, 0)
        typed = compile_source(deck(source))
        self.assertIsNone(typed.material.rate_type)
        self.assertEqual(declaration_report(typed)['interpretation']['rate_type'], 'unresolved_blank')
        self.assertIn('blank VP remains unresolved',
                      declaration_report(typed)['interpretation']['donor_options'])
        self.assertEqual(dict(typed.hardening_curve.documented_defaults),
                         {'sfa': 1., 'sfo': 1., 'offa': 0., 'offo': 0.})

    def test_later_duplicate_of_any_selected_reference_fails(self):
        for family in ('part', 'section', 'material', 'curve'):
            with self.subTest(family=family), self.assertRaisesRegex(WallImportError, 'duplicate'):
                source = blocks()
                compile_source(deck(source, tail=source[family]))

    def test_missing_selected_reference_fails(self):
        for family in ('section', 'material', 'curve'):
            with self.subTest(family=family), self.assertRaisesRegex(WallImportError, 'unresolved'):
                source = blocks()
                del source[family]
                compile_source(deck(source))

    def test_unknown_option_for_selected_identity_fails(self):
        for family in ('part', 'section', 'material', 'curve'):
            with self.subTest(family=family), self.assertRaisesRegex(WallImportError, 'unsupported keyword'):
                source = blocks()
                source[family][0] += '_UNSUPPORTED'
                compile_source(deck(source))

    def test_title_option_cannot_hide_later_duplicate(self):
        source = blocks()
        duplicate = ['*MAT_024_TITLE', 'another title'] + source['material'][1:]
        with self.assertRaisesRegex(WallImportError, 'duplicate material'):
            compile_source(deck(source, tail=duplicate))

    def test_nonblank_unsupported_fields_and_extra_cards_fail(self):
        changes = [
            ('part', 3, row(17, 27, 37, 99)),
            ('section', 1, row(27, 2, .8333, 3)),
            ('section', 2, row(1.648, 1.648, 1.648, 1.648, 0)),
            ('material', 2, row(8000, 8, 47, None, 0, 1)),
            ('material', 4, row(.1)),
            ('curve', 1, row(47, 0, 1, 1, None, None, 0)),
        ]
        for family, line, value in changes:
            with self.subTest(family=family, line=line), self.assertRaises(WallImportError):
                source = blocks()
                source[family][line] = value
                compile_source(deck(source))
        source = blocks()
        source['section'].append('')
        with self.assertRaises(WallImportError):
            compile_source(deck(source))

    def test_flags_curve_order_transforms_and_nonfinite_are_rejected(self):
        changes = [
            ('material', 2, row(8000, 8, 47, None, '.1')),
            ('material', 2, row(8000, 8, 47, None, 'NaN')),
            ('material', 2, row(8000, 8, 47, None, '1e9999999')),
            ('material', 2, row(8000, 8, 47, None, 1)),
            ('material', 1, row(37, 'nan', 200000, .3, 270)),
            ('material', 1, row(37, '7.89e-9', '1e308', .3, 270)),
            ('curve', 1, row(47, 0, 2, 1)),
            ('curve', 2, row(.01, 270, width=20)),
            ('curve', 3, row(0, 340, width=20)),
            ('curve', 3, row(.2, float('inf'), width=20)),
            ('curve', 4, row(.3, 300, width=20)),
        ]
        for family, line, value in changes:
            with self.subTest(family=family, value=value), self.assertRaises(WallImportError):
                source = blocks()
                source[family][line] = value
                compile_source(deck(source))

    def test_required_fields_and_units_are_not_inferred(self):
        source = blocks()
        source['material'][1] = row(37, None, 200000, .3, 270)
        with self.assertRaises(WallImportError):
            compile_source(deck(source))
        for scale in (0, -1, float('inf'), float('nan')):
            with self.subTest(scale=scale), self.assertRaises(ValueError):
                UnitSystem('t', 'mm', 's', scale, .001, 1)

    def test_source_structure_caps_and_create_only_report(self):
        for raw in (deck().replace(b'*END\n', b''), deck() + b'*PART\n',
                    b'*KEYWORD\n' + b'$' * 4097 + b'\n*END\n'):
            with self.subTest(raw=raw[:32]), self.assertRaises(WallImportError):
                compile_source(raw)
        report = declaration_report(compile_source(deck()))
        before = asdict(compile_source(deck()))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'declarations.json'
            write_report(path, report)
            saved = path.read_bytes()
            with self.assertRaises(FileExistsError):
                write_report(path, report)
            self.assertEqual(path.read_bytes(), saved)
        self.assertEqual(asdict(compile_source(deck())), before)


if __name__ == '__main__':
    unittest.main()
