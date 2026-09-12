#!/usr/bin/env python3
"""Root-only airbag80 fixture using the existing authenticated app source reader."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('app-root', 'assets', 'source-member', 'output'):
        parser.add_argument('--' + name, required=True, type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.app_root.resolve()))
    from modelio.solid_working_geometry import select_geometry, collect_working_solids
    from modelio.solid_geometry_export import _arrays
    from modelio.source_blocks import scan_declarations
    from modelio._legacy import require
    receipt_path = Path(__file__).with_name('source-receipt.json')
    receipt = json.loads(receipt_path.read_text())
    part = receipt['part']
    require(not args.output.exists() and args.output.parent.is_dir(), 'create-only output required')
    manifest, nodes, cells, arrays = select_geometry(args.assets, [2000945], receipt['canonical_sha256'])
    with args.source_member.open('rb') as stream:
        index = scan_declarations(stream, 'yaris-coarse-v1l.key')
    member = receipt['source_files']['yaris-coarse-v1l.key']['sha256']
    require(index.sha256 == member, 'original main member identity')
    blocks = []
    for family in ('part', 'section', 'material'):
        block = index.one(family, 2000945)
        require(block.sha256 == part[family]['sha256'], 'original declaration identity')
        blocks.append(dict(family=family, identity=2000945, **asdict(block)))
    require(set(cells) == set(part['source_element_ids']), 'complete original80 EIDs required')
    with args.source_member.open('rb') as stream:
        nodes, cells, summary = collect_working_solids(stream, nodes, cells, [2000945])
    require(summary['sha256'] == member and len(nodes) == 156 and len(cells) == 80, 'source geometry coverage')
    binary = _arrays(nodes, cells)
    require(hashlib.sha256(binary['solid_records_u64'][0]).hexdigest() == part['raw_ordered_records_sha256'],
            'all eight ordered source slots must match')
    report = dict(schema='tlfea.airbag-solid-geometry.v1', simulation_ready=False,
                  canonical_manifest_sha256=receipt['canonical_sha256'], member_sha256=member,
                  source_receipt_sha256=hashlib.sha256(receipt_path.read_bytes()).hexdigest(),
                  source_declarations=blocks, original_archive=manifest['source_archive'], input_arrays=arrays,
                  solid_count=80, node_count=156, repeated_slot_count=4,
                  original_formulation=receipt['original_formulation'],
                  selected_demo_formulation=receipt['selected_demo_formulation'], arrays={})
    for name, (_, descriptor) in binary.items():
        report['arrays'][name] = dict(file=name + '.bin', **descriptor)
    raw = (json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + '\n').encode('ascii')
    require(len(raw) + sum(len(b) for b, _ in binary.values()) <= 1024 * 1024, 'fixture byte cap')
    args.output.mkdir()
    for name, (value, _) in binary.items():
        with (args.output / (name + '.bin')).open('xb') as stream:
            stream.write(value)
    with (args.output / 'manifest.json').open('xb') as stream:
        stream.write(raw)
    print(json.dumps(dict(solids=80, nodes=156, repeated=4,
                         manifest_sha256=hashlib.sha256(raw).hexdigest())))


if __name__ == '__main__':
    main()
