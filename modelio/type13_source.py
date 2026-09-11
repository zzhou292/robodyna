"""Bounded startup-only source package for all original PID2000486 beam welds."""
from dataclasses import asdict
import json
from pathlib import Path
import zipfile

from ._legacy import file_sha256, require, sha256
from .canonical_geometry import load_array, read_json
from .source_blocks import scan_declarations, MAX_SOURCE_BYTES
from .type13_coordinates import collect_working_coordinates
from .type13_declarations import parse_beam_property, TYPE13_POLICY
from .yaris_part import REFERENCE, VEHICLE, _member

PART_ID = 2000486


def _canonical_selection(assets, manifest):
    specs = [('node_ids', 'Q', '<u8', 1), ('node_positions', 'd', '<f8', 3),
             ('node_source_lines', 'I', '<u4', 1), ('node_blank_masks', 'H', '<u2', 1),
             ('node_codes', 'i', '<i4', 2), ('beams_records', 'Q', '<u8', 10),
             ('beams_source_lines', 'I', '<u4', 1), ('beams_blank_masks', 'H', '<u2', 1)]
    require(sum(manifest['arrays'][name]['bytes'] for name, *_ in specs) <= 32 * 1024 * 1024,
            'TYPE13 canonical input byte cap exceeded')
    arrays = {name: load_array(assets, manifest, name, code, dtype, width) for name, code, dtype, width in specs}
    rows = arrays['beams_records']; beams = {}
    for i in range(len(rows) // 10):
        record = tuple(rows[10*i:10*i+10])
        if record[1] != PART_ID:
            continue
        require(record[0] not in beams and record[2] != record[3] and record[4] == 2000001 and
                record[5:9] == (0, 0, 0, 0) and record[9] == 2 and arrays['beams_blank_masks'][i] == 480,
                'TYPE13 source beam identity/N3/releases/LOCAL changed')
        beams[record[0]] = dict(source_id=record[0], canonical_index=i,
            source_line=arrays['beams_source_lines'][i], raw_record=record, blank_mask=arrays['beams_blank_masks'][i])
    wanted = {nid for row in beams.values() for nid in row['raw_record'][2:5]}
    require(len(beams) == 4442 and len(wanted) == 7494, 'Original TYPE13 source extent changed')
    nodes = {}
    for i, nid in enumerate(arrays['node_ids']):
        if nid in wanted:
            require(nid not in nodes, 'Duplicate canonical TYPE13 node')
            nodes[nid] = dict(source_line=arrays['node_source_lines'][i],
                blank_mask=arrays['node_blank_masks'][i], position_m=tuple(arrays['node_positions'][3*i:3*i+3]),
                codes=tuple(arrays['node_codes'][2*i:2*i+2]))
    require(set(nodes) == wanted, 'TYPE13 canonical source node closure is incomplete')
    return nodes, beams, {name: manifest['arrays'][name] for name, *_ in specs}


def compile_type13_startup(archive_path, asset_dir):
    archive_path, assets = Path(archive_path), Path(asset_dir)
    reference = read_json(REFERENCE)
    require(archive_path.stat().st_size <= MAX_SOURCE_BYTES and
            file_sha256(archive_path) == reference['archive_sha256'], 'TYPE13 source archive identity changed')
    require(file_sha256(assets / 'manifest.json') == 'c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8',
            'TYPE13 canonical manifest identity changed')
    manifest = read_json(assets / 'manifest.json')
    nodes, beams, arrays = _canonical_selection(assets, manifest)
    prefix = reference['archive_member_prefix']
    with zipfile.ZipFile(archive_path) as archive:
        _member(archive, prefix + 'README.md', 64 * 1024)
        _member(archive, prefix + VEHICLE, MAX_SOURCE_BYTES)
        readme = archive.read(prefix + 'README.md')
        require(sha256(readme) == 'f5cd0934d505e0bfd81de95d867e1b2614d804499e130d39885083af5c52349e',
                'TYPE13 original README unit authority changed')
        with archive.open(prefix + VEHICLE) as stream:
            index = scan_declarations(stream, VEHICLE)
        require(index.sha256 == reference['files'][VEHICLE]['sha256'], 'TYPE13 property source identity changed')
        property_value = parse_beam_property(index.one('part', PART_ID), index.one('section', PART_ID),
                                             index.one('material', PART_ID))
        with archive.open(prefix + VEHICLE) as stream:
            working, beam_rows, summary = collect_working_coordinates(stream, nodes, beams, PART_ID)
        require(summary == manifest['source_files'][VEHICLE], 'TYPE13 complete original source ledger changed')
    node_order = {nid: i for i, nid in enumerate(sorted(working))}
    ordered_beams = [dict(row, node_indices=[node_order[n] for n in row['raw_record'][2:5]])
                     for row in sorted(beam_rows.values(), key=lambda row: row['canonical_index'])]
    return dict(schema='robo_dyna.type13_source_startup.v1', simulation_ready=False,
        scope='Original PID2000486 startup only; tied endpoint pairing and recurrence remain unqualified',
        source=dict(archive_sha256=reference['archive_sha256'], member_sha256=index.sha256,
                    member_bytes=index.source_bytes, member=prefix + VEHICLE,
                    unit_authority=dict(raw_text=readme.decode('ascii'), sha256=sha256(readme)),
                    canonical_manifest_sha256=file_sha256(assets / 'manifest.json'), arrays=arrays),
        policy=TYPE13_POLICY, units=dict(mass_to_kg=1000., length_to_m=.001, time_to_s=1.),
        property=dict(part=asdict(property_value.part),
            section=dict(source=asdict(property_value.section_source), cards=[asdict(c) for c in property_value.section_cards]),
            material=dict(source=asdict(property_value.material_source), cards=[asdict(c) for c in property_value.material_cards])),
        nodes=[asdict(working[n]) for n in sorted(working)],
        beams=ordered_beams, counts=dict(nodes=7494, beams=4442, physical_endpoints=7493))


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--assets', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args()
    document = compile_type13_startup(options.archive, options.assets)
    encoded = (json.dumps(document, indent=2, allow_nan=False) + '\n').encode('ascii')
    require(len(encoded) <= 8 * 1024 * 1024, 'TYPE13 declaration output exceeds byte cap')
    with options.output.open('xb') as stream:
        stream.write(encoded)
    print(json.dumps(dict(file=str(options.output), bytes=len(encoded), sha256=sha256(encoded))))


if __name__ == '__main__':
    main()
