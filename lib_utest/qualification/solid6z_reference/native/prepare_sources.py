"""Authenticate and namespace complete selected S6Z geometry/mass leaves."""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent


def prepare(output, check):
    manifest = json.loads((ROOT/'source-manifest.json').read_text())
    record = manifest['license']
    value = (ROOT/record['path']).read_bytes()
    if len(value) != record['bytes'] or hashlib.sha256(value).hexdigest() != record['sha256']:
        raise RuntimeError('Native license changed')
    sources = {}
    for record in manifest['sources']:
        value = (ROOT/record['path']).read_bytes()
        blob = hashlib.sha1(b'blob '+str(len(value)).encode()+b'\0'+value).hexdigest()
        if (len(value) != record['bytes'] or hashlib.sha256(value).hexdigest() != record['sha256']
                or blob != record['git_blob_sha1']):
            raise RuntimeError('Native donor changed: '+record['source'])
        sources[Path(record['source']).name] = value.decode('latin1')
    names = {'constant_mod','precision_mod','element_mod','message_mod','checkvolume_8n',
             'checkvolume_6n','checkvolume_4n','srepiso3','sortho3','slen',
             's6zcoor3','s6zrcoor3','s6zjacidp','s6zderi3','s6zortho3',
             's6zcoor3_mod','s6zrcoor3_mod','s6zjacidp_mod','s6zderi3_mod','s6zortho3_mod',
             's6cortho3','s6fraca','s6mass3'}
    for value in sources.values():
        names.update(n.lower() for n in re.findall(r'COMMON\s*/\s*(\w+)\s*/',value,re.I))
    shared = {'ale_mod','q1np_restart_mod','my_exit'}
    pattern = re.compile(r'\b(?:'+'|'.join(sorted(names|shared,key=len,reverse=True))+r')\b',re.I)
    for name,value in sources.items():
        transformed = pattern.sub(lambda m: ('SOLID18_REF_' if m.group().lower() in shared
                                             else 'SOLID6Z_REF_')+m.group().upper(),value).encode('latin1')
        path = output/name
        if check:
            if not path.is_file() or path.read_bytes() != transformed:
                raise RuntimeError('Prepared donor changed: '+name)
        else:
            output.mkdir(parents=True,exist_ok=True)
            path.write_bytes(transformed)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--check',action='store_true')
    args = parser.parse_args()
    prepare(args.output,args.check)
