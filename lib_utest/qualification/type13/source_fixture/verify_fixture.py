"""Check the retained original-coordinate fixture bytes and frozen authority."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parent
manifest_bytes = (root / 'source-manifest.json').read_bytes()
assert hashlib.sha256(manifest_bytes).hexdigest() == '5bd53c28648f51566f119b9c2bed07ead464f36040b0a9382bee9a9909822095'
manifest = json.loads(manifest_bytes)
expected = '063b3208f45b5249308a29ba442e84bfa518b5bd502b3a0e0ad197ebfdfd11b8'
data = (root / manifest['file']).read_bytes()
assert manifest['sha256'] == hashlib.sha256(data).hexdigest() == expected
assert len(data) == manifest['bytes'] == 1418632
assert (manifest['beams'], manifest['nodes'], manifest['physical_endpoint_nodes']) == (4442, 7494, 7493)
assert manifest['source']['member_sha256'] == '67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301'
assert manifest['working_units'] == dict(mass_to_kg=1000, length_to_m=.001, time_to_s=1)
print('Authenticated original TYPE13 fixture: 4442 beams, 7494 original-coordinate nodes')
