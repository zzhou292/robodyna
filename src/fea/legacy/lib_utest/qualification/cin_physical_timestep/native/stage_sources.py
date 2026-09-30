"""Verify complete pinned donors and stage only the exact selected equations."""
import hashlib
import json
from pathlib import Path
import sys

own = Path(__file__).resolve().parent
cache, destination = map(Path, sys.argv[1:])
manifest = json.loads((own / 'source-manifest.json').read_text())
text = (own / 'Native.in.F90').read_text()
for item in manifest['files']:
    data = (cache / item['path']).read_bytes()
    assert len(data) == item['bytes']
    assert hashlib.sha256(data).hexdigest() == item['sha256']
    assert hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest() == item['git_blob_sha1']
    for fragment in item.get('fragments', []):
        exact = b''.join(data.splitlines(keepends=True)[fragment['first_line']-1:fragment['last_line']])
        assert hashlib.sha256(exact).hexdigest() == fragment['sha256']
        text = text.replace('@' + fragment['name'] + '@', exact.decode().rstrip('\n'))
assert '@' not in text
assert hashlib.sha256((own / 'Native.in.F90').read_bytes()).hexdigest() == manifest['wrapper_sha256']
destination.mkdir(parents=True, exist_ok=True)
(destination / 'Native.F90').write_text(text)
(destination / 'source-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('Complete DTNODA/RGBODFP and rigid-material audit donors verified; three exact fragments staged')
