#!/usr/bin/env python3
"""Authenticate complete native sources and selected exact setup boundaries."""
import argparse,hashlib,json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def prepare(output,check):
    manifest=json.loads((ROOT/'source-manifest.json').read_text())
    sources={}
    for entry in manifest['sources']:
        data=(ROOT/entry['path']).read_bytes()
        blob=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
        if len(data)!=entry['bytes'] or hashlib.sha256(data).hexdigest()!=entry['sha256'] or blob!=entry['git_blob_sha1']:
            raise ValueError('Native source changed: '+entry['path'])
        sources[entry['source']]=data.decode('latin1')
    names={'constant_mod','message_mod','names_and_titles_mod','element_mod','pcoori','peveci','pmass','defbeam_sect'}
    pattern=re.compile(r'\b(?:'+'|'.join(sorted(names,key=len,reverse=True))+r')\b',re.I)
    outputs={Path(name).name:text for name,text in sources.items() if Path(name).name in {'pcoori.F','peveci.F','pmass.F','constant_mod.F','my_real.inc'}}
    for section in ['complete_subroutines','selected_setup']:
        for name,(source,first,last) in manifest[section].items():
            value=''.join(sources[source].splitlines(keepends=True)[first-1:last])
            if section=='complete_subroutines' and not (re.match(r'\s*SUBROUTINE',value,re.I) and re.search(r'(?im)^\s*END SUBROUTINE DEFBEAM_SECT\s*$',value)):
                raise ValueError('Selected native subroutine boundary changed')
            outputs[name]=value
    for name,text in outputs.items():
        data=pattern.sub(lambda m:'BEAM18_REF_'+m.group().upper(),text).encode('latin1')
        path=output/name
        if check:
            if not path.is_file() or path.read_bytes()!=data:raise ValueError('Prepared source changed: '+name)
        else:
            output.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
    print('Beam18 complete native sources and exact selected setup authenticated')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--check',action='store_true');a=p.parse_args();prepare(a.output,a.check)
