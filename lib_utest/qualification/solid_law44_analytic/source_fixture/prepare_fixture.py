#!/usr/bin/env python3
"""Bounded binary artifact validation; no original source parsing or solver defaults."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import struct


def prepare(directory, expected, output):
    raw = (directory / 'manifest.json').read_bytes()
    if len(raw) > 1024 * 1024 or hashlib.sha256(raw).hexdigest() != expected:
        raise ValueError('airbag fixture manifest differs from explicit root receipt')
    m = json.loads(raw)
    receipt_path = Path(__file__).with_name('source-receipt.json')
    receipt = json.loads(receipt_path.read_text())
    if (m['schema'] != 'tlfea.airbag-solid-geometry.v1' or m['solid_count'] != 80 or m['node_count'] != 156 or
            m['source_receipt_sha256'] != hashlib.sha256(receipt_path.read_bytes()).hexdigest() or
            m['canonical_manifest_sha256'] != receipt['canonical_sha256'] or
            m['member_sha256'] != receipt['source_files']['yaris-coarse-v1l.key']['sha256'] or
            m['selected_demo_formulation'] != receipt['selected_demo_formulation'] or
            m['original_formulation'] != receipt['original_formulation']):
        raise ValueError('source or selected demo formulation receipt differs')
    # Reuse the complete established binary field/type inventory; only fixed row counts differ.
    path = Path(__file__).parents[2] / 'solid18_law44_reference/prepare_fixture.py'
    spec = importlib.util.spec_from_file_location('rear_fixture', path)
    shared = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(shared)
    if set(m['arrays']) != set(shared.LAYOUT):
        raise ValueError('complete binary array inventory required')
    arrays = {}
    for name, (code, dtype, _, columns) in shared.LAYOUT.items():
        rows = 156 if name.startswith('node_') else 80
        entry = m['arrays'][name]
        size = rows * columns * struct.calcsize('<' + code)
        if (entry['file'] != name + '.bin' or entry['dtype'] != dtype or
                entry['shape'] != [rows, columns] or entry['bytes'] != size):
            raise ValueError('binary array layout differs: ' + name)
        path = directory / entry['file']
        if path.stat().st_size != size:
            raise ValueError('binary extent differs: ' + name)
        raw = path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != entry['sha256']:
            raise ValueError('binary hash differs: ' + name)
        arrays[name] = struct.unpack('<' + code * (rows * columns), raw)
    ids = arrays['node_ids_u64']
    mm, si = arrays['node_position_mm_f64'], arrays['node_position_m_f64']
    if len(set(ids)) != 156 or any(n <= 0 for n in ids) or any(
            struct.pack('<d', a * .001) != struct.pack('<d', b) for a, b in zip(mm, si)):
        raise ValueError('node identity or one-way coordinate bits differ')
    records, local = arrays['solid_records_u64'], arrays['solid_nodes_local_u32']
    if (list(records[0::10]) != receipt['part']['source_element_ids'] or
            hashlib.sha256(struct.pack('<' + 'Q' * len(records), *records)).hexdigest() !=
            receipt['part']['raw_ordered_records_sha256']):
        raise ValueError('ordered original80 source slots differ')
    lines = ['#pragma once', '#include "lib_src/elements/solid18/law44/Reference.h"',
             'namespace law44_airbag_fixture {', 'namespace law = tl::fea::solid18::law44;',
             'struct Node { std::uint64_t nid; tl::fea::solid18::Vec3 mm, si; };',
             'inline constexpr Node Nodes[]={']
    for i, nid in enumerate(ids):
        fields = lambda row: ','.join(v.hex() for v in row)
        lines.append('{' + str(nid) + ',{' + fields(mm[3*i:3*i+3]) + '},{' + fields(si[3*i:3*i+3]) + '}},')
    lines += ['};', 'struct Cell { std::uint64_t eid; unsigned local[8]; };', 'inline constexpr Cell Cells[]={']
    repeated = []
    for i in range(80):
        eid, pid, *nodes = records[10*i:10*i+10]
        slots = local[8*i:8*i+8]
        if pid != 2000945 or any(n >= 156 for n in slots) or tuple(ids[n] for n in slots) != tuple(nodes):
            raise ValueError('source node/domain slot association differs')
        if len(set(nodes)) == 6 and nodes[4] == nodes[5] and nodes[6] == nodes[7]:
            repeated.append(eid)
        elif len(set(nodes)) != 8:
            raise ValueError('unsupported source repeated topology')
        lines.append('{' + str(eid) + ',{' + ','.join(map(str, slots)) + '}},')
    if repeated != [2167690, 2167709, 2167737, 2167756]:
        raise ValueError('four exact collapsed source identities differ')
    lines += ['};', 'inline law::ReferenceInput Input(unsigned i, bool working=false) {',
              '  law::ReferenceInput input{}; input.profile=law::Profile();',
              '  input.source_element_id=Cells[i].eid;',
              '  input.source_part_id=input.source_section_id=input.source_material_id=2000945;',
              '  input.density_kg_m3=working ? 1.95e-9 : 1.95e-9*1e12;',
              '  for(unsigned n=0;n<8;++n) { const auto& node=Nodes[Cells[i].local[n]];',
              '    input.source_node_id[n]=node.nid; input.position_m[n]=working ? node.mm : node.si; }',
              '  return input;', '}', '}']
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text('\n'.join(lines) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', required=True, type=Path)
    parser.add_argument('--manifest-sha256', required=True)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    prepare(args.fixture, args.manifest_sha256, args.output)
