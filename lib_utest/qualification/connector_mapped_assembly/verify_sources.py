"""Frozen serial assembly identity; only includes and namespace are adapted."""
import hashlib,json,os,pathlib
root=pathlib.Path(__file__).resolve().parent
for name,record in json.loads((root/'serial-source.json').read_text()).items():
 data=(root/name).read_bytes()
 if hashlib.sha256(data).hexdigest()!=record['adapted_sha256']:
  raise SystemExit('Frozen original assembly differs: '+name)
 text=data.decode().replace('::batch_detail::serial_reference','::batch_detail')
 lines=[]
 for line in text.splitlines():
  if line.startswith('#include "'):
   line='#include "'+os.path.relpath(line.split('"')[1],pathlib.Path(record['source']).parent)+'"'
  lines.append(line)
 original='\n'.join(lines)+'\n'
 if hashlib.sha256(original.encode()).hexdigest()!=record['source_sha256']:
  raise SystemExit('Reference adaptation changed numerical source: '+name)
print('Frozen TYPE13 and TYPE25 connector serial references match unchanged numerical source.')
