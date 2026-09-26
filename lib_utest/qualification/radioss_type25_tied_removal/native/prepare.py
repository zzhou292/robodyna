#!/usr/bin/env python3
"""Whole original PRE_I2/REMN_I2OP/UPGRADE_REMNODE2; qualification only."""
import argparse
import hashlib
import importlib.util
import json
import re
from pathlib import Path
ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('search_donors',ROOT.parents[1]/'radioss_type25_search_startup/native/Sources.py')
helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
NAMES={'MY_ALLOC_MOD':'TR_MEMORY','MY_MOVE_ALLOC_MOD':'TR_MOVE','MESSAGE_MOD':'TR_MESSAGES',
 'INTBUFDEF_MOD':'TR_BUFFER','NAMES_AND_TITLES_MOD':'TR_NAMES','FORMAT_MOD':'TR_FORMAT',
 'RESTMOD':'TR_RESTART','INTBUFMOD':'TR_INTBUF', 'REMN_I2OP':'TR_REMN_I2OP','PRE_I2':'TR_PRE_I2',
 'ZERONM_TAGD':'TR_ZERONM_TAGD','UPGRADE_REMNODE2':'TR_UPGRADE_REMNODE2',
 'FRETITL2':'TR_FRETITL2','REMN_I2OP_EDG25':'TR_EDGE_UNSUPPORTED','MY_ORDERS':'TR_ORDERS'}
def routine(source,name):
    start=re.search(r'^      SUBROUTINE '+name+r'\(',source,re.M)
    assert start,name
    tail=source[start.start():]
    end=re.search(r'^[ \t]+END[ \t]*$',tail,re.M)
    assert end,name
    body=tail[:end.end()]+'\n'
    assert len(re.findall(r'^[ \t]+SUBROUTINE\b',body,re.M))==1,name
    return body
def namespace(text):
    for old,new in NAMES.items():text=re.sub(r'\b'+old+r'\b',new,text,flags=re.I)
    return text
def generated():
    source={}
    for e in json.loads((ROOT/'source-manifest.json').read_text())['files']:
        data=(ROOT.parent/e['path']).read_bytes()
        assert len(data)==e['bytes'] and hashlib.sha256(data).hexdigest()==e['sha256']
        assert hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()==e['git_blob']
        source[Path(e['path']).name]=data.decode()
    bodies={name+'.F':routine(source['i7remnode.F'],name) for name in ['REMN_I2OP','PRE_I2','ZERONM_TAGD']}
    bodies['UPGRADE_REMNODE2.F']=routine(source['upgrade_remnode.F'],'UPGRADE_REMNODE2')
    result={name:namespace(text) for name,text in bodies.items()}
    # Names only. Reversing the one-to-one token substitution must recover every
    # original numerical body byte, including all merge and inverse-table code.
    for name,text in result.items():
        back=text
        for old,new in NAMES.items():
            # Original USE/call token case is preserved separately by canonical
            # tokenization; executable expressions and whitespace stay exact.
            back=re.sub(r'\b'+new+r'\b',old,back,flags=re.I)
        canonical=lambda s: re.sub(r'\b(?:'+ '|'.join(NAMES)+r')\b',lambda m:m.group().upper(),s,flags=re.I)
        assert canonical(back)==canonical(bodies[name])
    order=source['my_orders.c']
    for a,b in [('tri_direct','tr_orders_direct'),('my_orders__','tr_orders__'),('my_orders_','tr_orders_'),('MY_ORDERS','TR_ORDERS'),('my_orders','tr_orders')]:
        order=re.sub(r'\b'+a+r'\b',b,order)
    result['Orders.c']=order
    result['Constants.F90']=helper.constants(source['constant_mod.F'],[s.upper() for s in bodies.values()]).replace('selection_constants','tr_constants')
    for name in ['Boundary.F90','Wrapper.F90']:result[name]=(ROOT/name).read_text()
    result['implicit_f.inc']='      USE ISO_C_BINDING\n      USE TR_CONSTANTS\n      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n'
    result['param_c.inc']='      INTEGER,PARAMETER::NPARI=200,LNOPT1=4,LTITR=1\n'
    result['com04_c.inc']='      INTEGER NUMNOD,NINTER\n      COMMON /TR_COUNTS/NUMNOD,NINTER\n'
    result['scr17_c.inc']=''
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--check',action='store_true');a=p.parse_args()
    for name,text in generated().items():
        path=a.output/name
        if a.check:assert path.read_text()==text
        else:path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text)
    print('Whole pinned tied-removal, allocation/reset and stable original sort prepared')
