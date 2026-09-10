"""Actual existing canonical importer/reader interoperability, no solver."""
from array import array
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

app, executable = Path(sys.argv[1]).resolve(), sys.argv[2]
sys.path.insert(0, str(app / 'tools'))
sys.path.insert(0, str(app))
from import_yaris_vehicle import write_array
from modelio.canonical_geometry import load_array

with tempfile.TemporaryDirectory(prefix='robo-canonical-array-') as directory:
    root = Path(directory)
    (root / 'arrays').mkdir()
    specs = {
        'u16': (array('H', [0, 65535]), '<u2', 1),
        'u32': (array('I', [0, 4294967295]), '<u4', 1),
        'u64': (array('Q', [2**64 - 1, 2**53 + 1]), '<u8', 1),
        'i32': (array('i', [-2**31, 2**31 - 1]), '<i4', 1),
        'f64': (array('d', [0., -0., 5e-324, 1.2345678901234567]), '<f8', 2),
        'empty': (array('d'), '<f8', 1),
    }
    manifest = {'arrays': {}}
    for name, (values, dtype, columns) in specs.items():
        file = 'arrays/' + name + '.bin'
        manifest['arrays'][name] = dict(file=file, **write_array(root / file, values, columns))
    (root / 'manifest.json').write_text(json.dumps(manifest))
    subprocess.run([executable, '--gtest_filter=BoundedArrays.CanonicalImporterInteroperability'],
                   env=dict(os.environ, ROBO_DYNA_CANONICAL_ARRAY_FIXTURE=str(root)), check=True)
    for name, (expected, dtype, columns) in specs.items():
        copied = json.loads((root / ('cpp-' + name + '.json')).read_text())
        actual = load_array(root, {'arrays': {name: copied}}, name, expected.typecode, dtype, columns, allow_empty=True)
        assert actual.tobytes() == expected.tobytes(), name
print('Canonical Python -> C++ -> canonical Python: all five scalar types and empty array PASS')
