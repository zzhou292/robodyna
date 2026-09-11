from dataclasses import asdict
from hashlib import sha256
import copy
import unittest
from pathlib import Path
from unittest.mock import patch
import test_vehicle_declarations as legacy_fixture
from test_canonical_part_geometry import row
from modelio.declarations import UnitSystem
from modelio.vehicle_declarations import compile_vehicle_declarations
from modelio.vehicle_section_resolution import compile_vehicle_section_resolution, _encoded
from modelio.vehicle_section_resolution_io import write_vehicle_section_resolution


class VehicleSectionResolution(legacy_fixture.VehicleDeclarations):
    def failed_inputs(self, unsupported=False):
        old = row(38, '7.8900E-9', '2.0000E+5', .3, 180).encode()
        new = row(38, '7.8900E-9', '2.0000E+5', .3, 180, None, 1).encode()
        raw = self.f.base_raw.replace(old, new)
        if unsupported:
            # ELFORM9/NIP1 remains an independent section obligation.
            raw = raw.replace(row(28, 16, None, 3).encode(), row(28, 9, None, 1).encode())
        self.f.rebuild(raw)
        return self.inputs()

    def test_separate_resolution_binds_exact_historical_plan_and_source_cards(self):
        index, scope = self.failed_inputs()
        units = UnitSystem('t', 'mm', 's', 1000, .001, 1)
        legacy = compile_vehicle_declarations(index, scope, units, {})
        original = copy.deepcopy(legacy)
        result = compile_vehicle_section_resolution(index, scope, units, {})
        self.assertEqual(legacy, original)
        self.assertEqual(legacy['counts']['supported_shells'], 2)
        self.assertEqual([r['status'] for r in result['parts']], ['existing', 'constant_failure'])
        self.assertEqual(result['counts']['failure_shells'], 2)
        self.assertEqual(result['source']['vehicle_plan_sha256'], sha256(_encoded(legacy)).hexdigest())
        resolved = result['constant_failure_declarations']['declarations']
        self.assertEqual(resolved['materials'][0]['failure_strain'], 1)
        self.assertEqual(resolved['materials'][0]['source'], asdict(index.one('material', 38)))
        self.assertNotIn('geometry', result)
        self.assertFalse(result['simulation_ready'])

    def test_unsupported_section_is_not_resolved_by_positive_fail(self):
        index, scope = self.failed_inputs(True)
        with self.assertRaisesRegex(ValueError, 'no ordinary constant-failure'):
            compile_vehicle_section_resolution(index, scope, UnitSystem('t', 'mm', 's', 1000, .001, 1), {})

    def test_sidecar_preflights_bytes_and_create_only_publication(self):
        index, scope = self.failed_inputs()
        result = compile_vehicle_section_resolution(index, scope, UnitSystem('t', 'mm', 's', 1000, .001, 1), {})
        path = Path(self.temp.name) / 'resolution.json'
        with patch('modelio.vehicle_section_resolution_io.MAX_BYTES', 10):
            with self.assertRaises(ValueError): write_vehicle_section_resolution(path, result)
        self.assertFalse(path.exists())
        write_vehicle_section_resolution(path, result)
        before = path.read_bytes()
        with self.assertRaises(FileExistsError): write_vehicle_section_resolution(path, result)
        self.assertEqual(path.read_bytes(), before)


if __name__ == '__main__': unittest.main()
