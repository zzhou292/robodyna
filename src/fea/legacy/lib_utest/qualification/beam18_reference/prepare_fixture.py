#!/usr/bin/env python3
"""Authenticate fixed original142 beam binaries and emit build-local hex values."""
import argparse,hashlib,json,math,struct
from pathlib import Path
EXPECTED='5d85548343e4190db4f6ebc5f0975f6e05921140c7403a87ba389c6445dd1644'
LAYOUT={'node_ids':('Q','<u8',147,1),'node_position_native':('d','<f8',147,3),
        'node_position_m':('d','<f8',147,3),'node_source_lines':('Q','<u8',147,1),
        'node_codes':('i','<i4',147,2),'node_blank_masks':('I','<u4',147,1),
        'beam_records':('Q','<u8',142,10),'beam_nodes_local':('I','<u4',142,3),
        'beam_source_lines':('Q','<u8',142,1),'beam_blank_masks':('I','<u4',142,1)}


def prepare(fixture,output):
    p=fixture/'manifest.json'
    if p.stat().st_size>2*1024*1024:raise ValueError('Fixture manifest size changed')
    data=p.read_bytes()
    if hashlib.sha256(data).hexdigest()!=EXPECTED:raise ValueError('Original fixture manifest changed')
    manifest=json.loads(data)
    if manifest['beam_count']!=142 or manifest['node_count']!=147 or set(manifest['arrays'])!=set(LAYOUT):
        raise ValueError('Original fixture inventory changed')
    arrays={}
    for name,(code,dtype,rows,columns) in LAYOUT.items():
        entry=manifest['arrays'][name];size=rows*columns*struct.calcsize('<'+code)
        if entry['file']!=name+'.bin' or entry['dtype']!=dtype or entry['shape']!=[rows,columns] or entry['bytes']!=size:
            raise ValueError('Original array descriptor changed')
        path=fixture/entry['file']
        if path.stat().st_size!=size:raise ValueError('Original array size changed')
        values=path.read_bytes()
        if hashlib.sha256(values).hexdigest()!=entry['sha256']:raise ValueError('Original array identity changed')
        arrays[name]=struct.unpack('<'+code*(rows*columns),values)
    ids=arrays['node_ids'];native=arrays['node_position_native'];si=arrays['node_position_m']
    if len(set(ids))!=147 or 0 in ids:raise ValueError('Original node identity changed')
    if any(not math.isfinite(x) or struct.pack('<d',x*.001)!=struct.pack('<d',y) for x,y in zip(native,si)):
        raise ValueError('Original native/SI coordinate association changed')
    records=arrays['beam_records'];local=arrays['beam_nodes_local'];masks=arrays['beam_blank_masks']
    counts={2000514:0,2000520:0,2000948:0,2000950:0}
    if len(set(records[0::10]))!=142:raise ValueError('Original EIDs changed')
    text=['#pragma once','#include "TestSupport.h"','namespace beam18_test::original {',
          'inline constexpr beam::Vec3 Positions[]={']
    for n in range(147):text.append('{'+','.join(x.hex() for x in native[3*n:3*n+3])+'},')
    text += ['};','struct Cell { std::uint64_t eid,pid,nodes[3]; unsigned local[3]; };','inline constexpr Cell Cells[]={']
    for n in range(142):
        eid,pid,n1,n2,n3,*options=records[10*n:10*n+10];slots=local[3*n:3*n+3]
        if pid not in counts or options!=[0,0,0,0,2] or masks[n]!=480 or any(s>=147 for s in slots):
            raise ValueError('Original beam selected profile changed')
        if tuple(ids[s] for s in slots)!=(n1,n2,n3) or len({n1,n2,n3})!=3:
            raise ValueError('Original endpoint/orientation source slots changed')
        counts[pid]+=1
        text.append('{'+f'{eid},{pid},'+'{'+f'{n1},{n2},{n3}'+'},{'+','.join(map(str,slots))+'}},')
    if counts!={2000514:36,2000520:36,2000948:35,2000950:35}:raise ValueError('Original part counts changed')
    text += ['};','inline beam::Input Input(unsigned row) {',
             '  const auto& cell=Cells[row]; beam::Input input{};',
             '  input.source_element_id=cell.eid;',
             '  input.source_part_id=input.source_section_id=input.source_material_id=cell.pid;',
             '  input.units=beam::WorkingUnits::TonneMillimetreSecond;',
             '  input.profile=beam::Profile::CircularFourPointStoredZero;',
             '  for(unsigned n=0;n<3;++n){input.source_node_id[n]=cell.nodes[n];input.position[n]=Positions[cell.local[n]];}',
             '  switch(cell.pid) {']
    for part in manifest['declarations']['parts']:
        material=part['material']['cards'][0]['text'];section=part['section']['cards'][1]['text']
        rho,e,nu=(float(material[k:k+10]) for k in (10,20,30))
        radius=max(float(section[:10])*.5,.5*float(section[10:20]))
        text.append(f'    case {part["pid"]}: input.radius={radius.hex()};input.density={rho.hex()};input.young={e.hex()};input.poisson={nu.hex()};break;')
    text += ['  }','  return input;','}','} // namespace beam18_test::original']
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text('\n'.join(text)+'\n')
    print('Original142 beam/147 node fixture authenticated; exact working values emitted')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--fixture',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();prepare(a.fixture,a.output)
