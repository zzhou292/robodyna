#!/usr/bin/env python3
"""Authenticate a fixed binary fixture and emit build-local exact C++ values."""
import argparse
import hashlib
import json
from pathlib import Path
import struct

EXPECTED = 'e799eee2fb93e7189fb1bbc84a7d100c6e1f98e50d381dd71ce5d2c2d354fd4b'
LAYOUT = {
    'node_ids_u64': ('Q', '<u8', 476, 1),
    'node_position_mm_f64': ('d', '<f8', 476, 3),
    'node_position_m_f64': ('d', '<f8', 476, 3),
    'node_source_line_u64': ('Q', '<u8', 476, 1),
    'node_codes_i64': ('q', '<i8', 476, 2),
    'node_blank_mask_u32': ('I', '<u4', 476, 1),
    'node_canonical_index_u32': ('I', '<u4', 476, 1),
    'solid_records_u64': ('Q', '<u8', 306, 10),
    'solid_nodes_local_u32': ('I', '<u4', 306, 8),
    'solid_source_line_u64': ('Q', '<u8', 306, 1),
    'solid_blank_mask_u32': ('I', '<u4', 306, 1),
    'solid_canonical_index_u32': ('I', '<u4', 306, 1),
}


def prepare(directory, output):
    raw = (directory/'manifest.json').read_bytes()
    if len(raw) > 2*1024*1024 or hashlib.sha256(raw).hexdigest() != EXPECTED:
        raise ValueError('original fixture manifest identity changed')
    manifest = json.loads(raw)
    if (manifest['schema'] != 'tlfea.rear-metal-solid-geometry.v1' or
            manifest['solid_count'] != 306 or manifest['node_count'] != 476 or
            set(manifest['arrays']) != set(LAYOUT)):
        raise ValueError('unexpected original geometry inventory')
    arrays = {}
    for name, (code, dtype, rows, columns) in LAYOUT.items():
        entry = manifest['arrays'][name]
        size = rows*columns*struct.calcsize('<'+code)
        if entry['file'] != name+'.bin' or entry['dtype'] != dtype or entry['shape'] != [rows, columns] or entry['bytes'] != size:
            raise ValueError('original array layout changed: '+name)
        path = directory/entry['file']
        if path.stat().st_size != size:
            raise ValueError('original array size changed: '+name)
        raw = path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != entry['sha256']:
            raise ValueError('original array identity changed: '+name)
        arrays[name] = struct.unpack('<'+code*(rows*columns), raw)
    ids = arrays['node_ids_u64']
    mm, si = arrays['node_position_mm_f64'], arrays['node_position_m_f64']
    if len(set(ids)) != 476 or any(n <= 0 for n in ids):
        raise ValueError('original node identity changed')
    if any(struct.pack('<d', x*.001) != struct.pack('<d', y) for x, y in zip(mm, si)):
        raise ValueError('original one-way coordinate bits changed')
    records, local = arrays['solid_records_u64'], arrays['solid_nodes_local_u32']
    if len(set(records[0::10])) != 306:
        raise ValueError('original EID identity changed')
    repeated = 0
    counts = {2000016: 0, 2000392: 0}
    lines = ['#pragma once', '#include "TestSupport.h"', 'namespace rear18_test::original {',
             'struct Node { std::uint64_t nid; s::Vec3 mm, si; };', 'inline constexpr Node Nodes[]={']
    for i, nid in enumerate(ids):
        fields = lambda row: ','.join(x.hex() for x in row)
        lines.append('{'+str(nid)+',{'+fields(mm[3*i:3*i+3])+'},{'+fields(si[3*i:3*i+3])+'}},')
    lines += ['};', 'struct Cell { std::uint64_t eid,pid; unsigned local[8]; };', 'inline constexpr Cell Cells[]={']
    for i in range(306):
        eid, pid, *nodes = records[10*i:10*i+10]
        slots = local[8*i:8*i+8]
        if any(n >= 476 for n in slots) or tuple(ids[n] for n in slots) != tuple(nodes) or pid not in counts:
            raise ValueError('original slot association changed')
        if len(set(nodes)) == 6:
            if nodes[4] != nodes[5] or nodes[6] != nodes[7] or len(set(nodes[:5]+[nodes[6]])) != 6:
                raise ValueError('original collapsed slot pattern changed')
            repeated += 1
        elif len(set(nodes)) != 8:
            raise ValueError('unsupported original source topology')
        counts[pid] += 1
        lines.append('{'+str(eid)+','+str(pid)+',{'+','.join(map(str, slots))+'}},')
    if counts != {2000016: 210, 2000392: 96} or repeated != 109:
        raise ValueError('original part/topology census changed')
    lines += ['};', 'inline s::ReferenceInput Input(unsigned i, bool working=false) {',
              '  s::ReferenceInput input{}; input.profile=law::Profile();',
              '  const auto& cell=Cells[i]; input.source_element_id=cell.eid;',
              '  input.source_part_id=input.source_section_id=input.source_material_id=cell.pid;',
              '  input.density_kg_m3=working ? 7.8900e-9 : 7.8900e-9*1e12;',
              '  for(unsigned n=0;n<8;++n) { const auto& node=Nodes[cell.local[n]];',
              '    input.source_node_id[n]=node.nid; input.position_m[n]=working ? node.mm : node.si; }',
              '  return input;', '}', '}']
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text('\n'.join(lines)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    prepare(args.fixture, args.output)
