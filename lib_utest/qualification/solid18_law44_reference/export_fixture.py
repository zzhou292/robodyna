#!/usr/bin/env python3
"""Root-only original rear geometry staging, using existing frozen app readers.

The TL production/qualification build does not import the app. It consumes only
this authenticated binary artifact; this offline staging tool is not a parser.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--app-root', required=True, type=Path)
    parser.add_argument('--assets', required=True, type=Path)
    parser.add_argument('--source-member', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.app_root.resolve()))
    from modelio.solid_working_geometry import select_geometry, collect_working_solids
    from modelio.solid_geometry_export import _arrays
    from modelio.source_blocks import scan_declarations
    from modelio.canonical_geometry import file_hash
    from modelio._legacy import require, sha256
    require(not args.output.exists() and args.output.parent.is_dir(), 'create-only fixture destination')
    receipt_path = Path(__file__).resolve().parents[1] / 'solid_law44_point/source_fixture/source-receipt.json'
    receipt = json.loads(receipt_path.read_text())
    pids = [2000016, 2000392]
    manifest, nodes, elements, arrays = select_geometry(args.assets, pids, receipt['canonical_sha256'])
    with args.source_member.open('rb') as stream:
        index = scan_declarations(stream, 'yaris-coarse-v1l.key')
    member_hash = receipt['source_files']['yaris-coarse-v1l.key']['sha256']
    require(index.sha256 == member_hash, 'original member identity')
    blocks = []
    for part in receipt['parts']:
        pid = part['pid']
        require({eid for eid, e in elements.items() if e['raw_record'][1] == pid} ==
                set(part['source_element_ids']), 'original EID coverage')
        for family in ('part', 'section', 'material'):
            block = index.one(family, pid)
            require(block.sha256 == part[family]['sha256'], 'original declaration identity')
            blocks.append(dict(family=family, identity=pid, **asdict(block)))
    curve = index.one('curve', 2100270)
    require(curve.sha256 == receipt['curve']['sha256'], 'original curve identity')
    blocks.append(dict(family='curve', identity=2100270, **asdict(curve)))
    with args.source_member.open('rb') as stream:
        nodes, elements, summary = collect_working_solids(stream, nodes, elements, pids)
    require(summary['sha256'] == member_hash and len(elements) == 306, 'source geometry identity')
    binary = _arrays(nodes, elements)
    repeated = sum(len(set(e['raw_record'][2:])) == 6 for e in elements.values())
    require(repeated == 109, 'original repeated-slot coverage')
    report = dict(schema='tlfea.rear-metal-solid-geometry.v1', simulation_ready=False,
                  canonical_manifest_sha256=receipt['canonical_sha256'], member_sha256=member_hash,
                  original_archive=manifest['source_archive'], source_declarations=blocks,
                  material_receipt_sha256=file_hash(receipt_path), input_arrays=arrays,
                  node_count=len(nodes), solid_count=306, repeated_slot_count=109,
                  source_length_unit='mm', output_length_unit='m', length_scale=.001,
                  density_source=7.8900e-9, density_to_si=1e12,
                  slot_order='original source N1..N8; no element permutation', arrays={})
    for name, (_, descriptor) in binary.items():
        report['arrays'][name] = dict(file=name+'.bin', **descriptor)
    data = (json.dumps(report, indent=2, sort_keys=True, allow_nan=False)+'\n').encode('ascii')
    require(len(data)+sum(len(b) for b, _ in binary.values()) <= 4*1024*1024, 'fixture byte cap')
    args.output.mkdir()
    for name, (value, _) in binary.items():
        with (args.output/(name+'.bin')).open('xb') as stream:
            stream.write(value)
    with (args.output/'manifest.json').open('xb') as stream:
        stream.write(data)
    print(json.dumps(dict(solids=306, nodes=len(nodes), repeated=109, manifest_sha256=sha256(data))))


if __name__ == '__main__':
    main()
