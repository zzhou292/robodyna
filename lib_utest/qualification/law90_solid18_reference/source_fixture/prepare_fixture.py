#!/usr/bin/env python3
"""Verify bounded original binary geometry; emit build-only hex-float arrays."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
MANIFEST_SHA='c5adae0a61384bb7b57c88ed113798cf5b0fbaa8758fd48f93e15cc61f65f94a'
N,E=2904,1345
SCHEMA={
 'node_ids_u64':('<u8',N,1), 'node_position_mm_f64':('<f8',N,3),
 'node_position_m_f64':('<f8',N,3), 'node_source_line_u64':('<u8',N,1),
 'node_codes_i64':('<i8',N,2), 'node_blank_mask_u32':('<u4',N,1),
 'node_canonical_index_u32':('<u4',N,1), 'solid_records_u64':('<u8',E,10),
 'solid_nodes_local_u32':('<u4',E,8), 'solid_source_line_u64':('<u8',E,1),
 'solid_blank_mask_u32':('<u4',E,1), 'solid_canonical_index_u32':('<u4',E,1),
}
def read_fixture(directory):
    manifest_path=directory/'manifest.json'
    if manifest_path.stat().st_size>128*1024: raise ValueError('manifest size')
    raw=manifest_path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=MANIFEST_SHA: raise ValueError('manifest identity')
    m=json.loads(raw)
    if set(m['arrays'])!=set(SCHEMA): raise ValueError('array inventory')
    a={}
    for key,(dtype,rows,columns) in SCHEMA.items():
        d=m['arrays'][key]
        if d['file']!=key+'.bin' or d['dtype']!=dtype or d['shape']!=[rows,columns]:
            raise ValueError('array shape/type/path: '+key)
        expected=rows*columns*int(dtype[-1])
        if d['bytes']!=expected or expected>128*1024: raise ValueError('array size')
        path=directory/d['file']
        if path.stat().st_size!=expected: raise ValueError('file size: '+key)
        data=path.read_bytes()
        if hashlib.sha256(data).hexdigest()!=d['sha256']: raise ValueError('array digest: '+key)
        code={'<u8':'Q','<u4':'I','<i8':'q','<f8':'d'}[dtype]
        values=struct.unpack('<'+str(rows*columns)+code,data)
        a[key]=[values[i*columns:(i+1)*columns] for i in range(rows)]
    ids=[row[0] for row in a['node_ids_u64']]
    if len(set(ids))!=N or min(ids)<=0: raise ValueError('node identity')
    for mm,si in zip(a['node_position_mm_f64'],a['node_position_m_f64']):
        for x,y in zip(mm,si):
            if not math.isfinite(x) or struct.pack('<d',x*.001)!=struct.pack('<d',y):
                raise ValueError('working to SI bits')
    eids=set()
    for record,local in zip(a['solid_records_u64'],a['solid_nodes_local_u32']):
        if record[0]<=0 or record[0] in eids or record[1]!=2000063 or len(set(local))!=8:
            raise ValueError('element identity/topology')
        eids.add(record[0])
        if any(n>=N for n in local) or tuple(ids[n] for n in local)!=record[2:]:
            raise ValueError('source/local node-slot agreement')
    for key in ['node_canonical_index_u32','solid_canonical_index_u32','solid_source_line_u64']:
        values=[r[0] for r in a[key]]
        if any(x>=y for x,y in zip(values,values[1:])): raise ValueError('source order')
    p=m['declarations']['parts']
    if len(p)!=1 or any(p[0][k]!=2000063 for k in ['part_id','section_id','material_id']):
        raise ValueError('source declarations')
    for key in ['density_tonne_per_mm3','density_kg_per_m3']:
        if struct.pack('<d',p[0][key]['value']).hex()!=p[0][key]['binary64_le']:
            raise ValueError('density bits')
    return m,a

def generate(directory,output,check=False):
    m,a=read_fixture(directory)
    p=m['declarations']['parts'][0]
    lines=['// Build-generated from authenticated original source. No element admission.',
        '#pragma once','#include <cstdint>','namespace radiator_source_fixture {',
        'inline constexpr unsigned node_count=2904, element_count=1345;',
        'inline constexpr std::uint64_t part_id=2000063,section_id=2000063,material_id=2000063;',
        'inline constexpr double density_si='+p['density_kg_per_m3']['value'].hex()+';',
        'inline constexpr double density_working='+p['density_tonne_per_mm3']['value'].hex()+';']
    for key,(dtype,rows,columns) in SCHEMA.items():
        ctype={'<u8':'std::uint64_t','<u4':'std::uint32_t','<i8':'std::int64_t','<f8':'double'}[dtype]
        lines.append('inline constexpr '+ctype+' '+key+'['+str(rows)+']['+str(columns)+']={')
        for row in a[key]:
            lines.append('{'+','.join(x.hex() if dtype=='<f8' else str(x) for x in row)+'},')
        lines.append('};')
    lines.append('} // namespace radiator_source_fixture')
    result=('\n'.join(lines)+'\n').encode()
    if check:
        if output.read_bytes()!=result: raise ValueError('generated fixture changed')
    else:
        output.parent.mkdir(parents=True,exist_ok=True)
        output.write_bytes(result)
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--check',action='store_true')
    a=p.parse_args(); generate(a.input,a.output,a.check)
