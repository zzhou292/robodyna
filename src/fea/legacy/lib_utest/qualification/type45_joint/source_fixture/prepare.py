"""Authenticate the original 44 joint cards and canonical SI geometry; no property resolution."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import zipfile

ROOT=Path(__file__).resolve().parent
CANONICAL='c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8'
KEY='yaris-coarse-v1l.key'
KINDS={'*CONSTRAINED_JOINT_SPHERICAL_ID':1,'*CONSTRAINED_JOINT_REVOLUTE_ID':2,
       '*CONSTRAINED_JOINT_CYLINDRICAL_ID':3}
EXCLUDED={2200514,2200515,2200526,2200527,2200528,2200529}

def prepare(assets,output,check):
    manifest_data=(assets/'manifest.json').read_bytes()
    if hashlib.sha256(manifest_data).hexdigest()!=CANONICAL:
        raise RuntimeError('Original canonical identity changed')
    manifest=json.loads(manifest_data)
    blocks=[b for b in manifest['source_files'][KEY]['blocks'] if b['keyword'] in KINDS]
    if len(blocks)!=44: raise RuntimeError('Original joint coverage changed')
    lines_to_keep={n for b in blocks for n in range(b['first_line'],b['last_line']+1)}
    lines={}
    digest=hashlib.sha256()
    with zipfile.ZipFile(assets/'source_model.zip') as archive:
        member='2010-toyota-yaris-coarse-v1l/'+KEY
        if archive.getinfo(member).file_size>64*1024*1024:
            raise RuntimeError('Original source key size changed')
        with archive.open(member) as stream:
            for n,line in enumerate(stream,1):
                digest.update(line)
                if n in lines_to_keep: lines[n]=line.decode().rstrip('\r\n')
    if digest.hexdigest()!=manifest['source_files'][KEY]['sha256']:
        raise RuntimeError('Original source key changed')
    rows=[]
    requested=set()
    for block in blocks:
        data=[(n,lines[n]) for n in range(block['first_line']+1,block['last_line']+1)
              if lines[n].strip() and not lines[n].startswith(('$','*'))]
        if len(data)!=2 or data[1][1][60:].strip(): raise RuntimeError('Original option blank changed')
        eid=int(data[0][1][:10])
        nodes=[int(data[1][1][10*i:10*(i+1)].strip() or '0') for i in range(5)]
        count=2 if KINDS[block['keyword']]==1 else 3
        if any(n<=0 for n in nodes[:count]): raise RuntimeError('Original joint node missing')
        requested.update(nodes[:count])
        rows.append((eid,KINDS[block['keyword']],data[1][0],nodes,data[1][1],count))
    arrays={}
    for name in ('node_ids','node_positions'):
        description=manifest['arrays'][name]
        path=assets/description['file']
        if path.stat().st_size!=description['bytes']:
            raise RuntimeError('Original coordinate array size changed')
        value=path.read_bytes()
        if len(value)!=description['bytes'] or hashlib.sha256(value).hexdigest()!=description['sha256']:
            raise RuntimeError('Original coordinate array changed')
        arrays[name]=value
    positions={}
    for i,(nid,) in enumerate(struct.iter_unpack('<Q',arrays['node_ids'])):
        if nid in requested: positions[nid]=struct.unpack_from('<ddd',arrays['node_positions'],24*i)
    if len(positions)!=len(requested): raise RuntimeError('Original joint coordinate missing')
    text=['// Generated original geometry only; explicitly supplied properties/context in tests.\n',
          '#pragma once\n#include <cstdint>\n#include "lib_src/math/Fixed3.h"\n',
          'namespace type45_source {\nstruct Row { std::uint64_t eid; unsigned kind,line; bool retained; ',
          'std::uint64_t raw_nodes[5]; tl::math::Vec3 positions[3]; const char* raw; };\n',
          'inline const Row rows[]{\n']
    for eid,kind,line,nodes,raw,count in rows:
        points=[positions[n] for n in nodes[:count]]+[(0.,0.,0.)]*(3-count)
        point_text=','.join('{'+','.join(v.hex() for v in point)+'}' for point in points)
        text.append('{'+f'{eid},{kind},{line},'+('false' if eid in EXCLUDED else 'true')+',{'+
                    ','.join(map(str,nodes))+'},{'+point_text+'},'+json.dumps(raw)+'},\n')
    text.append('};\n}\n')
    data=''.join(text).encode()
    path=output/'OriginalJoints.h'
    if check:
        if not path.is_file() or path.read_bytes()!=data: raise RuntimeError('Prepared joint geometry changed')
    else:
        output.mkdir(parents=True,exist_ok=True)
        path.write_bytes(data)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    prepare(args.assets,args.output,args.check)
