"""Create-only, binary selected-solid source evidence. No geometry permutation."""
from array import array
import json
from pathlib import Path
import sys

from ._legacy import require, sha256
from .canonical_geometry import read_json, file_hash, _int, VEHICLE
from .source_blocks import scan_declarations, MAX_SOURCE_BYTES
from .solid_working_geometry import select_geometry, collect_working_solids
from .solid_geometry_evidence import declaration_evidence

MAX_EXPORT_BYTES = 4 * 1024 * 1024
REFERENCE = Path(__file__).resolve().parents[1] / 'models/yaris_coarse_v1l.json'


def _binary(code, rows, columns, names):
    values = array(code, (value for row in rows for value in row))
    size = {'Q': 8, 'I': 4, 'd': 8, 'q': 8}[code]
    require(values.itemsize == size and len(values) % columns == 0, 'unsupported binary representation')
    if sys.byteorder != 'little':
        values.byteswap()
    data = values.tobytes()
    dtype = {'Q': '<u8', 'I': '<u4', 'd': '<f8', 'q': '<i8'}[code]
    return data, dict(dtype=dtype, shape=[len(values)//columns, columns], bytes=len(data),
                      sha256=sha256(data), fields=names)


def _arrays(nodes, elements):
    n = sorted(nodes.values(), key=lambda v: v['canonical_index'])
    e = sorted(elements.values(), key=lambda v: v['source_line'])
    local = {row['canonical_index']: i for i, row in enumerate(n)}
    node_ids = [nid for nid, _ in sorted(nodes.items(), key=lambda kv: kv[1]['canonical_index'])]
    specs = {
        'node_ids_u64': ('Q', [(nid,) for nid in node_ids], 1, ['NID']),
        'node_position_mm_f64': ('d', [v['position_native'] for v in n], 3, ['x_mm', 'y_mm', 'z_mm']),
        'node_position_m_f64': ('d', [v['position_m'] for v in n], 3, ['x_m', 'y_m', 'z_m']),
        'node_source_line_u64': ('Q', [(v['source_line'],) for v in n], 1, ['source_line']),
        'node_codes_i64': ('q', [v['codes'] for v in n], 2, ['TC', 'RC']),
        'node_blank_mask_u32': ('I', [(v['blank_mask'],) for v in n], 1, ['blank_mask']),
        'node_canonical_index_u32': ('I', [(v['canonical_index'],) for v in n], 1, ['canonical_index']),
        'solid_records_u64': ('Q', [v['raw_record'] for v in e], 10,
                              ['EID', 'PID'] + ['N'+str(i) for i in range(1, 9)]),
        'solid_nodes_local_u32': ('I', [tuple(local[i] for i in v['canonical_node_indices']) for v in e],
                                  8, ['local'+str(i) for i in range(1, 9)]),
        'solid_source_line_u64': ('Q', [(v['source_line'],) for v in e], 1, ['source_line']),
        'solid_blank_mask_u32': ('I', [(v['blank_mask'],) for v in e], 1, ['blank_mask']),
        'solid_canonical_index_u32': ('I', [(v['canonical_index'],) for v in e], 1, ['canonical_index']),
    }
    return {name: _binary(*spec) for name, spec in specs.items()}


def export_geometry(assets, source_member, output, part_ids, manifest_sha256, *,
                    reference_path=REFERENCE, parent_cap=4096, node_cap=8192,
                    expected_parents=None, expected_nodes=None, byte_cap=MAX_EXPORT_BYTES):
    """Validate completely before claiming a new directory; manifest is last.

    The supplied local member is authenticated directly against the original
    model reference. Archive identity is provenance, not a claim that this call
    reopened or authenticated the ZIP container itself.
    """
    _int(byte_cap, 1, MAX_EXPORT_BYTES, 'export byte cap')
    output, source_member = Path(output), Path(source_member)
    require(not output.exists(), 'geometry destination already exists')
    require(output.parent.is_dir(), 'geometry destination parent must exist')
    reference_path = Path(reference_path)
    reference = read_json(reference_path)
    require(source_member.is_file() and source_member.stat().st_size <= MAX_SOURCE_BYTES,
            'source member missing or oversized')
    manifest, nodes, elements, inputs = select_geometry(
        assets, part_ids, manifest_sha256, parent_cap=parent_cap, node_cap=node_cap)
    if expected_parents is not None:
        require(len(elements) == _int(expected_parents, 1, parent_cap, 'expected solid count'),
                'selected solid count mismatch')
    if expected_nodes is not None:
        require(len(nodes) == _int(expected_nodes, 1, node_cap, 'expected node count'),
                'selected node count mismatch')
    member_hash = reference['files'][VEHICLE]['sha256']
    require(manifest['source_archive']['sha256'] == reference['archive_sha256'] and
            manifest['source_files'][VEHICLE]['sha256'] == member_hash,
            'canonical/original reference identity mismatch')
    with source_member.open('rb') as stream:
        index = scan_declarations(stream, VEHICLE)
    require(index.sha256 == member_hash, 'original source member SHA256 mismatch')
    declarations = declaration_evidence(index, manifest, part_ids)
    with source_member.open('rb') as stream:
        nodes, elements, summary = collect_working_solids(stream, nodes, elements, part_ids)
    require(summary['sha256'] == member_hash, 'geometry source member SHA256 changed')
    arrays = _arrays(nodes, elements)
    report = dict(schema='robo-dyna.selected-solid-working-geometry.v1', simulation_ready=False,
                  geometry_modified=False, source_geometry_validated=True,
                  part_ids=list(part_ids), node_count=len(nodes), solid_count=len(elements),
                  node_order='ascending original canonical node index',
                  solid_order='ascending original source line; all eight original slots retained',
                  source_length_unit='mm', output_length_unit='m', length_scale=.001,
                  applied_rigid_transform=None, source_mass_unit='t', density_to_si=1e12,
                  source=dict(canonical_manifest_sha256=manifest_sha256,
                              reference_sha256=file_hash(reference_path),
                              original_archive_sha256=reference['archive_sha256'],
                              archive_member=reference['archive_member_prefix'] + VEHICLE,
                              member_sha256=member_hash, member_bytes=index.source_bytes,
                              archive_container_verified_by_exporter=False),
                  canonical_arrays=inputs, declarations=declarations, arrays={})
    app = Path(__file__).resolve().parents[1]
    generator = ['modelio/' + name + '.py' for name in
                 ('solid_working_geometry', 'solid_geometry_export', 'solid_geometry_evidence',
                  'canonical_geometry', 'canonical_incidence', 'source_blocks', 'keyword_cards', '_legacy')]
    generator += ['tools/' + name + '.py' for name in
                  ('import_yaris_vehicle', 'import_yaris_wall', 'export_solid_working_geometry')]
    report['generator_sources'] = {name: file_hash(app / name) for name in generator}
    for name, (_, spec) in arrays.items():
        report['arrays'][name] = dict(file=name + '.bin', **spec)
    encoded = (json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + '\n').encode('ascii')
    require(len(encoded) + sum(len(data) for data, _ in arrays.values()) <= byte_cap,
            'geometry export byte cap exceeded')
    output.mkdir()  # Atomic create-only claim; a failed write leaves no success manifest.
    for name, (data, _) in arrays.items():
        with (output / (name + '.bin')).open('xb') as stream:
            stream.write(data)
    with (output / 'manifest.json').open('xb') as stream:
        stream.write(encoded)
    return report
