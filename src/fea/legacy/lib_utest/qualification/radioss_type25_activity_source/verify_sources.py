from pathlib import Path
import hashlib,json
here=Path(__file__).resolve().parent
root=here.parents[2]
manifest=json.loads((here/'inherited-source.json').read_text())
for name,pin in manifest['files'].items():
    data=(root/name).read_bytes()
    assert len(data)==pin['bytes'] and hashlib.sha256(data).hexdigest()==pin['sha256'],name
print(f"{len(manifest['files'])} inherited physical-source files verified")
