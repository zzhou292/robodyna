"""Tiny typed card fixtures; not authenticated vehicle/native startup evidence."""
import json
from pathlib import Path
import sys
import tempfile

root = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(root), str(root / 'tests')]
from test_source_assembly import AssemblyFixture
from test_glass_declarations import glass_fixture, UNITS
from modelio.vehicle_section_resolution import compile_vehicle_section_resolution

with tempfile.TemporaryDirectory() as directory:
    fixture = AssemblyFixture(directory)
    values = {}
    for name, nloc in (('centered', None), ('bottom', -1), ('top', 1)):
        index, scope = glass_fixture(fixture, nloc)
        values[name] = compile_vehicle_section_resolution(index, scope, UNITS, {}, include_glass=True)
    Path(sys.argv[1]).write_text(json.dumps(values, allow_nan=False))
