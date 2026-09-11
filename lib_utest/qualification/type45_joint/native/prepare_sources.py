"""Authenticate complete pinned joint leaves; only namespace and context adaptation."""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parent
COMPLETE={'rini45.F','rini45_rb.F','rskew33.F','ruser33.F','rdtime33.F','rcum33.F',
          'constant_mod.F','my_real.inc'}

def prepare(output,check):
    manifest=json.loads((ROOT/'source-manifest.json').read_text())
    if manifest['revision']!='a62b27e6baa555d222a580d6218867d0be4d70b5':
        raise RuntimeError('Native revision changed')
    sources={}
    for row in manifest['sources']:
        data=(ROOT/row['path']).read_bytes()
        blob=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
        if len(data)!=row['bytes'] or hashlib.sha256(data).hexdigest()!=row['sha256'] or blob!=row['git_blob_sha1']:
            raise RuntimeError('Native donor changed: '+row['source'])
        sources[Path(row['source']).name]=data.decode('latin1')
    row=manifest['license']
    license_data=(ROOT/row['path']).read_bytes()
    license_blob=hashlib.sha1(b'blob '+str(len(license_data)).encode()+b'\0'+license_data).hexdigest()
    if (len(license_data)!=row['bytes'] or hashlib.sha256(license_data).hexdigest()!=row['sha256'] or
        license_blob!=row['git_blob_sha1']):
        raise RuntimeError('Native license changed')
    prepared={key:value for key,value in sources.items() if key in COMPLETE}
    # Complete contiguous per-joint source block: main-node decode, equations,
    # warnings and all native type assignments. No numerical statement edits.
    prepared['automatic.inc']=''.join(sources['joint_block_stiffness.F'].splitlines(keepends=True)[133:280])
    prepared['automatic_alpha.inc']=sources['joint_block_stiffness.F'].splitlines(keepends=True)[102]
    names={'message_mod','names_and_titles_mod','element_mod','sensor_mod','constant_mod',
           'get_u_geo','get_u_pnu','get_u_func','get_u_func_deri','get_u_skew','fretitl2'}
    for value in prepared.values():
        names.update(name.lower() for name in re.findall(r'^\s*(?:INTEGER\s+)?(?:SUBROUTINE|FUNCTION)\s+(\w+)',value,re.I|re.M))
    pattern=re.compile(r'\b(?:'+'|'.join(sorted(names,key=len,reverse=True))+r')\b',re.I)
    for name,value in prepared.items():
        value=pattern.sub(lambda m:'T45_'+m.group().upper(),value)
        if name!='my_real.inc':
            # Fixed-form legal comment adaptation of the donor copyright header.
            value=''.join('!'+line if line.startswith('Copyright>') else line
                          for line in value.splitlines(keepends=True))
        path=output/name
        data=value.encode('latin1')
        if check:
            if not path.is_file() or path.read_bytes()!=data:
                raise RuntimeError('Prepared source changed: '+name)
        else:
            output.mkdir(parents=True,exist_ok=True)
            path.write_bytes(data)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    prepare(args.output,args.check)
