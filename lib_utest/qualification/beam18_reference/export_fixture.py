#!/usr/bin/env python3
"""Create the original142 beam fixture using the existing authenticated app readers.

Qualification staging only. No TL production dependency on the application.
The existing per-part working-coordinate collector is reused unchanged.
"""
import argparse
from array import array
import hashlib
import json
from pathlib import Path
import sys

CANONICAL = 'c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8'
MEMBER = '67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301'
DECLARATIONS = '3ef2b1f94a5bd40fcb3eb9d19a22fb44ebccf34f773bf5b77abf2c78f8393c5c'
PARTS = {2000514:36,2000520:36,2000948:35,2000950:35}


def export(args):
    sys.path.insert(0,str(args.app))
    from modelio.canonical_geometry import load_array,read_json,file_hash
    from modelio.type13_coordinates import collect_working_coordinates
    if args.output.exists():
        raise ValueError('Fixture output already exists')
    if (file_hash(args.assets/'manifest.json') != CANONICAL or
            file_hash(args.source) != MEMBER or file_hash(args.declarations) != DECLARATIONS):
        raise ValueError('Original source/canonical/declaration identity changed')
    manifest = read_json(args.assets/'manifest.json')
    declarations = read_json(args.declarations)
    arrays = {}
    def read(name,code,dtype,columns):
        arrays[name] = manifest['arrays'][name]
        return load_array(args.assets,manifest,name,code,dtype,columns)
    records = read('beams_records','Q','<u8',10)
    lines = read('beams_source_lines','I','<u4',1)
    masks = read('beams_blank_masks','H','<u2',1)
    selected = [(i,tuple(records[10*i:10*i+10])) for i in range(len(lines)) if records[10*i+1] in PARTS]
    if len(selected) != 142 or len({row[0] for _,row in selected}) != 142:
        raise ValueError('Original142 beam coverage changed')
    ids = read('node_ids','Q','<u8',1)
    positions = read('node_positions','d','<f8',3)
    node_lines = read('node_source_lines','I','<u4',1)
    node_codes = read('node_codes','i','<i4',2)
    node_masks = read('node_blank_masks','H','<u2',1)
    wanted = {nid for _,row in selected for nid in row[2:5]}
    nodes = {nid:{'source_line':node_lines[i], 'blank_mask':node_masks[i],
                 'codes':tuple(node_codes[2*i:2*i+2]), 'position_m':tuple(positions[3*i:3*i+3])}
             for i,nid in enumerate(ids) if nid in wanted}
    if 0 in wanted or set(nodes) != wanted or len(nodes) != 147:
        raise ValueError('Original147 endpoint/orientation node coverage changed')
    working = {}; source_rows = {}
    for pid,count in PARTS.items():
        rows = {row[0]:{'raw_record':row,'source_line':lines[i],'blank_mask':masks[i]}
                for i,row in selected if row[1] == pid}
        if len(rows) != count: raise ValueError('Original part count changed')
        subset = {n:nodes[n] for row in rows.values() for n in row['raw_record'][2:5]}
        with args.source.open('rb') as stream:
            actual_nodes,actual_rows,_ = collect_working_coordinates(stream,subset,rows,pid)
        for nid,node in actual_nodes.items():
            if nid in working and working[nid] != node: raise ValueError('Shared original node changed')
            working[nid] = node
        source_rows.update(actual_rows)
    if set(working) != wanted or set(source_rows) != {row[0] for _,row in selected}:
        raise ValueError('Working source coverage changed')
    ordered = sorted(working.values(),key=lambda node:node.source_line)
    local = {node.source_id:i for i,node in enumerate(ordered)}
    descriptors = {}; payloads = {}
    def add(name,code,columns,values):
        values = array(code,values)
        if sys.byteorder != 'little': values.byteswap()
        data = values.tobytes(); payloads[name+'.bin'] = data
        descriptors[name] = {'file':name+'.bin','dtype':'<'+{'Q':'u8','I':'u4','i':'i4','d':'f8'}[code],
            'shape':[len(values)//columns,columns],'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()}
    add('node_ids','Q',1,(n.source_id for n in ordered))
    add('node_position_native','d',3,(v for n in ordered for v in n.position_native))
    add('node_position_m','d',3,(v for n in ordered for v in n.position_m))
    add('node_source_lines','Q',1,(n.source_line for n in ordered))
    add('node_codes','i',2,(v for n in ordered for v in n.codes))
    add('node_blank_masks','I',1,(n.blank_mask for n in ordered))
    add('beam_records','Q',10,(v for _,r in selected for v in r))
    add('beam_nodes_local','I',3,(local[n] for _,r in selected for n in r[2:5]))
    add('beam_source_lines','Q',1,(lines[i] for i,_ in selected))
    add('beam_blank_masks','I',1,(masks[i] for i,_ in selected))
    result = {'schema':'tlfea.beam18-original-geometry.v1','source_units':'tonne millimetre second',
        'node_count':147,'beam_count':142,'part_counts':PARTS,'canonical_sha256':CANONICAL,
        'member_sha256':MEMBER,'declarations_sha256':DECLARATIONS,'declarations':declarations,
        'arrays':descriptors,'input_arrays':arrays,'source_slots':'N1,N2 endpoints; N3 orientation only',
        'material_or_runtime_admission':False}
    args.output.mkdir(parents=True)
    for name,data in payloads.items(): (args.output/name).open('xb').write(data)
    (args.output/'manifest.json').open('x').write(json.dumps(result,indent=2)+'\n')
    print('Created original142 beam /147 node fixture',args.output)
    print('manifest_sha256',file_hash(args.output/'manifest.json'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['app','assets','source','declarations','output']:
        parser.add_argument('--'+name,type=Path,required=True)
    export(parser.parse_args())
