from dataclasses import asdict
from io import BytesIO
from pathlib import Path
import copy
import tempfile
import unittest

from test_source_assembly import AssemblyFixture
from test_canonical_part_geometry import row
from modelio.declarations import UnitSystem
from modelio.source_blocks import scan_declarations
from modelio.glass_declarations import glass_material, glass_section, compile_glass_part
from modelio.vehicle_section_resolution import compile_vehicle_section_resolution, _encoded
from modelio.vehicle_declarations import compile_vehicle_declarations

UNITS = UnitSystem('t', 'mm', 's', 1000, .001, 1)


def glass_fixture(fixture, nloc=None, numint=1):
    index = scan_declarations(BytesIO(fixture.base_raw), 'yaris-coarse-v1l.key')
    material = '\n'.join(('*MAT_MODIFIED_PIECEWISE_LINEAR_PLASTICITY',
        row(38, '2.5000E-9', 70000, .22, 30, 1000, .015),
        row(None, None, 0, 0, None, None, None, numint), '', '')) + '\n'
    section = '\n'.join(('*SECTION_SHELL', row(28, 2, None, 3),
                         row(2.28, 2.28, 2.28, 2.28, nloc))) + '\n'
    raw = fixture.base_raw.replace(index.one('material', 38).raw_text.encode(), material.encode())
    raw = raw.replace(index.one('section', 28).raw_text.encode(), section.encode())
    fixture.rebuild(raw)
    index = scan_declarations(BytesIO(fixture.raw), 'yaris-coarse-v1l.key')
    tables = {kind: [] for kind in ('material', 'section', 'curve')}
    for (kind, identity), blocks in index.entries.items():
        if kind in tables:
            tables[kind].append(dict(identity=identity, sha256=blocks[0].sha256))
    parts = [dict(source_part_id=p, source_material_id=p+20, source_section_id=p+10,
                  source_part_sha256=index.one('part', p).sha256,
                  retained_shell_part=True, counts=dict(shells=2)) for p in (17, 18)]
    scope = dict(schema='robo-dyna.full-shell-scope.v1', source=dict(member_sha256=index.sha256),
                 selected_shell_part_ids=[17, 18], declarations=dict(parts=parts, tables=tables),
                 coverage=dict(retained=dict(shells=4)))
    return index, scope


class GlassDeclarations(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.fixture = AssemblyFixture(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def test_three_reference_planes_preserve_raw_cards_and_explicit_native_policy(self):
        for nloc, placement in ((None, 'centered'), (-1, 'bottom_reference_plane'), (1, 'top_reference_plane')):
            index, scope = glass_fixture(self.fixture, nloc)
            result = compile_vehicle_section_resolution(index, scope, UNITS, {}, include_glass=True)
            self.assertEqual(result['schema'], 'robo-dyna.vehicle-section-resolution.v2')
            self.assertEqual([r['status'] for r in result['parts']], ['existing', 'glass_tab1'])
            self.assertEqual(result['counts']['glass_shells'], 2)
            self.assertEqual(result['counts']['placed_glass_shells'], 0 if nloc is None else 2)
            typed = result['glass_declarations']['parts'][0]
            self.assertEqual(typed['section']['placement'], placement)
            self.assertEqual(typed['section']['source_nloc'], nloc)
            self.assertEqual(typed['material']['source'], asdict(index.one('material', 38)))
            self.assertEqual(typed['material']['cards'][1]['values'][:2], (None, None))
            self.assertIsNone(typed['material']['cards'][1]['values'][4])
            self.assertEqual(typed['material']['young_pa'], 70e9)
            self.assertEqual(typed['material']['density_kg_m3'], 2.5e-9*(1000/.001**3))
            self.assertEqual(result['glass_declarations']['policy']['resolved_VP'], 2)
            self.assertFalse(result['simulation_ready'])

    def test_original_v1_and_source_bits_remain_unchanged(self):
        index, scope = glass_fixture(self.fixture)
        original = compile_vehicle_declarations(index, scope, UNITS, {})
        before = _encoded(original)
        compile_vehicle_section_resolution(index, scope, UNITS, {}, include_glass=True)
        self.assertEqual(_encoded(compile_vehicle_declarations(index, scope, UNITS, {})), before)
        self.assertEqual(original['parts'][1]['status'], 'unresolved')
        with self.assertRaisesRegex(ValueError, 'no ordinary constant-failure'):
            compile_vehicle_section_resolution(index, scope, UNITS, {})
        for parser in (self.fixture.compile_assembly,):
            with self.assertRaises(ValueError):
                parser(material_policy='layered_law1_or_law44')

    def test_unsupported_numint_nloc_fld_and_nonblank_rate_remain_unresolved(self):
        for nloc, numint in ((.5, 1), (None, 2), (None, None)):
            index, scope = glass_fixture(self.fixture, nloc, numint)
            with self.assertRaises(ValueError):
                compile_vehicle_section_resolution(index, scope, UNITS, {}, include_glass=True)
        index, _ = glass_fixture(self.fixture)
        block = index.one('material', 38)
        from dataclasses import replace
        for field, value in ((0, 0), (4, 0), (5, 1), (6, .1), (2, 47), (3, 48)):
            values = [None, None, 0, 0, None, None, None, 1]
            values[field] = value
            cards = list(block.cards)
            cards[1] = replace(cards[1], text=row(*values))
            with self.assertRaises(ValueError):
                glass_material(replace(block, cards=tuple(cards)), UNITS)

    def test_stale_source_identity_rejects_without_mutation_then_clean_retry(self):
        index, scope = glass_fixture(self.fixture, -1)
        before = copy.deepcopy(scope)
        scope['declarations']['tables']['material'][-1]['sha256'] = '0'*64
        with self.assertRaises(ValueError):
            compile_vehicle_section_resolution(index, scope, UNITS, {}, include_glass=True)
        self.assertEqual(compile_vehicle_section_resolution(index, before, UNITS, {}, include_glass=True)
                         ['counts']['glass_shells'], 2)

    def test_wrong_units_and_late_invalid_thickness_do_not_produce_declaration(self):
        index, _ = glass_fixture(self.fixture)
        with self.assertRaises(ValueError):
            compile_glass_part(index, 18, UnitSystem('t','mm','ms',1000,.001,.001))
        from dataclasses import replace
        block = index.one('section',28)
        cards = (block.cards[0], replace(block.cards[1], text=row(2.28,2.28,2.28,0)))
        with self.assertRaises(ValueError):
            glass_section(replace(block,cards=cards),UNITS)


if __name__ == '__main__':
    unittest.main()
