"""Verify complete pinned leaves; namespace symbols and extract exact interfaces.

Native expressions remain unchanged. SZHOUR receives two observation-only
calls copying its work and coefficients; they never feed engine variables.
Interfaces come from the same complete leaves, retaining leading dimensions.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parent
LEAVES={'szderi3.F','szhour3.F','szetfac.F','sordeft3.F','sztorth3.F',
        'sgcoor3.F','sdefot3.F','sdefo3.F','sdlen3.F','slen.F','sfint3.F',
        'srepiso3.F','sortho3.F','srrota3.F','schkjabt3.F'}
INTERFACES={'SZDERI3','SZDERITO3','SZHOUR3','SORDEFT3','SZTORTH3',
            'SGCOOR3','SDEFOT3','SDEFO3','SDLEN3','SFINT3','SREPISO3','SORTHO3','SRROTA3'}

def interface(source,name):
    lines=source.splitlines(keepends=True)
    start=next(i for i,s in enumerate(lines) if re.search(r'^\s*SUBROUTINE\s+'+name+r'\(',s,re.I))
    end=start
    while ')' not in lines[end]:end+=1
    dummy=next(i for i in range(end,len(lines)) if 'D u m m y' in lines[i])
    local=next(i for i in range(dummy,len(lines)) if 'L o c a l' in lines[i])
    uses=[s for s in lines[end+1:dummy] if re.match(r'\s*USE\s',s,re.I)]
    declarations=lines[dummy+2:local-1]
    # The source common parameters determine SZHOUR's PM/GEO/PARTSAV extents.
    extra='#include "param_c.inc"\n' if name=='SZHOUR3' else ''
    return (''.join(lines[start:end+1])+''.join(uses)+
            '#include "implicit_f.inc"\n#include "mvsiz_p.inc"\n'+extra+
            ''.join(declarations)+'      END SUBROUTINE\n')

def prepare(output,check):
    manifest=json.loads((ROOT/'source-manifest.json').read_text())
    if manifest['revision']!='a62b27e6baa555d222a580d6218867d0be4d70b5':
        raise RuntimeError('Native revision changed')
    record=manifest['license'];data=(ROOT/record['path']).read_bytes()
    if len(data)!=record['bytes'] or hashlib.sha256(data).hexdigest()!=record['sha256']:
        raise RuntimeError('Native license changed')
    sources={};names={'constant_mod','precision_mod','element_mod','message_mod',
        'ale_mod','alefvm_mod','aleanim_mod','elbufdef_mod','matparam_def_mod',
        'arret','slena','sldege','szsvm','mdama24','szstrainhg'}
    for record in manifest['sources']:
        data=(ROOT/record['path']).read_bytes()
        blob=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
        if (len(data)!=record['bytes'] or blob!=record['git_blob_sha1'] or
                hashlib.sha256(data).hexdigest()!=record['sha256']):
            raise RuntimeError('Native source changed: '+record['source'])
        text=data.decode('latin1')
        if Path(record['source']).name in LEAVES or '/share/' in record['source'] or '/modules/' in record['source']:
            sources[Path(record['source']).name]=text
            names.update(x.lower() for x in re.findall(r'^\s*SUBROUTINE\s+(\w+)',text,re.M|re.I))
            names.update(x.lower() for x in re.findall(r'COMMON\s*/\s*(\w+)\s*/',text,re.I))
    declarations='      MODULE HEPH_NATIVE_INTERFACES\n      INTERFACE\n'
    for name in sorted(INTERFACES):
        source=next(s for s in sources.values() if re.search(r'^\s*SUBROUTINE\s+'+name+r'\(',s,re.M|re.I))
        declarations+=interface(source,name)
    sources['NativeInterfaces.F']=declarations+'      END INTERFACE\n      END MODULE\n'
    hour=sources['szhour3.F'].splitlines(keepends=True)
    if 'EINT(I)= EINT(I)+DT05*DEINT(I)' not in hour[461]:
        raise RuntimeError('First hourglass observation boundary changed')
    first='      CALL HEPH_NATIVE_HOUR_OBSERVE(1,I,DT05*DEINT(I),GG(I),FCL(I))\n'
    # The source last line already closes the power sum; retain that close.
    second='      CALL HEPH_NATIVE_HOUR_OBSERVE(2,I,DT05*(\n'+''.join(hour[681:687]).rstrip()+',GG(I),FCL(I))\n'
    sources['szhour3.F']=''.join(hour[:462])+first+''.join(hour[462:688])+second+''.join(hour[688:])
    pattern=re.compile(r'\b(?:'+'|'.join(sorted(names,key=len,reverse=True))+r')\b',re.I)
    for name,text in sources.items():
        data=pattern.sub(lambda m:'HEPH_NATIVE_'+m.group().upper(),text).encode('latin1')
        path=output/name
        if check:
            if not path.is_file() or path.read_bytes()!=data:
                raise RuntimeError('Prepared source changed: '+name)
        else:
            output.mkdir(parents=True,exist_ok=True);path.write_bytes(data)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args();prepare(args.output,args.check)
