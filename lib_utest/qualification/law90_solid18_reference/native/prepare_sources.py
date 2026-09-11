#!/usr/bin/env python3
"""Authenticate complete pinned donors and extract exact selected native stages.

Wrappers supply storage and observable packing only. Starter and engine symbols
have separate namespaces; no equation replacement or production inputs occur.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
ROOT = Path(__file__).resolve().parent
REVISION = 'a62b27e6baa555d222a580d6218867d0be4d70b5'
S = 'starter/source/elements/'
E = 'engine/source/elements/solid/'
ENGINE_LEAVES = {'s8ederic3.F','s8ejacip3.F','s8ederipr3.F','s8ederig3.F',
    's8eselecsh.F','s8ederish2.F','s8ea2bp8.F','s8eprst_ini.F','s8edefo3.F',
    'schkjabt3.F','srepiso3.F','sortho3.F','srrota3.F','sgcoor3.F',
    'sordeft3.F','s8edefot3.F','s8egetpij.F','s8eselecsht.F'}
SLICES = {
 'connectivity.inc': (S+'solid/solide/srcoor3.F',106,137),
 'gather.inc': (S+'solid/solide/srcoor3.F',143,168),
 'double_coordinates.inc': (S+'solid/solide/srcoor3.F',249,274),
 'frame.inc': (S+'solid/solide/srcoor3.F',282,300),
 'project.inc': (S+'solid/solide/srcoor3.F',304,383),
 'saved_tags.inc': (S+'elbuf_init/elbuf_ini.F',790,803),
 'gauss_data.inc': (S+'solid/solide8z/s8zinit3.F',181,238),
 'allocation_tags.inc': (S+'elbuf_init/elbuf_ini.F',818,836),
 'allocate_local_jac.inc': (S+'elbuf_init/allocbuf_auto.F',258,259),
 'allocate_local_pij.inc': (S+'elbuf_init/allocbuf_auto.F',275,276),
 'allocate_global_saved.inc': (S+'elbuf_init/allocbuf_auto.F',370,371),
 'allocate_global_jac.inc': (S+'elbuf_init/allocbuf_auto.F',398,399),
}
def authenticate():
    m=json.loads((ROOT/'source-manifest.json').read_text())
    if m['revision']!=REVISION: raise ValueError('revision changed')
    record=m['license']; value=(ROOT/record['path']).read_bytes()
    if len(value)!=record['bytes'] or hashlib.sha256(value).hexdigest()!=record['sha256']:
        raise ValueError('license identity')
    data={}
    for r in m['sources']:
        value=(ROOT/r['path']).read_bytes()
        if len(value)!=r['bytes'] or hashlib.sha256(value).hexdigest()!=r['sha256'] or hashlib.sha1(b'blob '+str(len(value)).encode()+b'\0'+value).hexdigest()!=r['git_blob_sha1']:
            raise ValueError('donor identity: '+r['source'])
        data[r['source']]=value.decode('latin1')
    return data

def subroutine(text,name):
    match=re.search(r'(?im)^\s*SUBROUTINE\s+'+name+r'\b',text)
    if not match: raise ValueError('missing subroutine '+name)
    end=re.search(r'(?im)^\s*END\s*$',text[match.start():])
    if not end: raise ValueError('unterminated subroutine '+name)
    return text[match.start():match.start()+end.end()]+'\n'

def prepare(output,check):
    data=authenticate()
    common_names={'constant_mod','precision_mod','element_mod','message_mod',
                  'elbufdef_mod','arret','checkvolume_8n','checkvolume_6n','checkvolume_4n'}
    for value in data.values():
        common_names.update(n.lower() for n in re.findall(r'COMMON\s*/\s*(\w+)\s*/',value,re.I))
    for stage in ['starter','engine']:
        outputs={}
        for source,value in data.items():
            basename=Path(source).name
            # Common CONSTANT_MOD and starter IMPLICIT_F need the same precision
            # and hardware macros; their authenticated owners are under engine.
            if (source.startswith(stage+'/share/') or source.startswith('common_source/') or
                    source in {'engine/share/r8/my_real.inc','engine/share/spe_inc/hardware.inc'}):
                if basename in outputs and outputs[basename]!=value: raise ValueError('ambiguous basename '+basename)
                outputs[basename]=value
            if stage=='engine' and source.startswith('engine/') and basename in ENGINE_LEAVES:
                outputs[basename]=value
        if stage=='engine':
            outputs['selection.inc']=''.join(data[E+'solide8e/s8eforc3.F'].splitlines(keepends=True)[515:526])
        if stage=='starter':
            for name in ['checksvolume.F','srepiso3.F','sortho3.F']:
                outputs[name]=data[S+'solid/solide/'+name]
            outputs['s8zderi3.F']=data[S+'solid/solide8z/s8zderi3.F']
            outputs['sgsavini.F']=subroutine(data[S+'solid/solide/scoor3.F'],'SGSAVINI')
            for name,(source,first,last) in SLICES.items():
                outputs[name]=''.join(data[source].splitlines(keepends=True)[first-1:last])
        for name,value in outputs.items():
            for include in re.findall(r'^\s*#include\s+"([^"]+)"',value,re.M):
                if include not in outputs:
                    raise ValueError('missing '+stage+' include '+include+' from '+name)
        names=set(common_names)
        for value in outputs.values():
            names.update(n.lower() for n in re.findall(r'(?im)^\s*(?:SUBROUTINE|(?:\w+\s+)*FUNCTION)\s+(\w+)',value))
        # JACOB_J33 occurs only in excluded ICP>0; fail if reached, no substitute.
        if stage=='engine': names.add('jacob_j33')
        tokens=re.compile(r'\b(?:'+'|'.join(sorted(names,key=len,reverse=True))+r')\b',re.I)
        prefix='LAW90_START_' if stage=='starter' else 'LAW90_ENGINE_'
        for name,value in outputs.items():
            result=tokens.sub(lambda m:prefix+m.group().upper(),value).encode('latin1')
            path=output/stage/name
            if check:
                if not path.is_file() or path.read_bytes()!=result: raise ValueError('prepared identity '+str(path))
            else:
                path.parent.mkdir(parents=True,exist_ok=True)
                path.write_bytes(result)
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--check',action='store_true')
    a=p.parse_args(); prepare(a.output,a.check)
