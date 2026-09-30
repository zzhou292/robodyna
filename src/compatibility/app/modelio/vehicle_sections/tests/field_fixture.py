"""Tiny declaration fixture only; it is not authenticated vehicle admission evidence."""
import json
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(root), str(root / 'tests')]
from test_vehicle_section_resolution import VehicleSectionResolution
from test_canonical_part_geometry import row
from modelio.declarations import UnitSystem
from modelio.vehicle_section_resolution import compile_vehicle_section_resolution

fixture = VehicleSectionResolution()
fixture.setUp()
try:
    fixture.failed_inputs()
    raw = fixture.f.raw.replace(row(38, '7.8900E-9', '2.0000E+5', .3, 180, None, 1).encode(),
                                row(38, '7.8900E-9', '2.0000E+5', .3, 180, 0., 1).encode())
    fixture.f.rebuild(raw)
    index, scope = fixture.inputs()
    units = UnitSystem('t', 'mm', 's', 1000, .001, 1)
    table = compile_vehicle_section_resolution(index, scope, units, {})
    raw = fixture.f.raw.replace(
        row(38, '7.8900E-9', '2.0000E+5', .3, 180, 0., 1).encode(),
        row(38, '7.8900E-9', '2.0000E+5', .3, 180, 800, 3.5).encode())
    # The fixture's second material references curve 48.
    raw = raw.replace(row(8000, 8, 48, None, '0.0').encode(), row(8000, 8, 0, None, 0).encode())
    fixture.f.rebuild(raw)
    index, scope = fixture.inputs()
    linear = compile_vehicle_section_resolution(index, scope, units, {})
    Path(sys.argv[1]).write_text(json.dumps(dict(table=table, linear=linear), allow_nan=False))
finally:
    fixture.tearDown()
