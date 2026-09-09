"""Bounded, source-verified shell selection from the existing canonical arrays.

No mesh conversion, source transform, mass, attachment or formulation admission.
The array reader is factored from planning/probe/yaris_shell_mesh_quality.py.
"""
from array import array
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import zipfile

from ._legacy import fields, file_sha256, require
from .source_blocks import MAX_LINE_BYTES, MAX_SOURCE_BYTES

file_hash = file_sha256  # Historical probe API.
MAX_ARRAY_BYTES = 64 * 1024 * 1024
VEHICLE = 'yaris-coarse-v1l.key'


def _int(value, minimum, maximum, label):
    require(isinstance(value, int) and not isinstance(value, bool) and minimum <= value <= maximum,
            'invalid ' + label)
    return value


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'duplicate JSON key: ' + key)
        result[key] = value
    return result


def read_json(path, cap=4 * 1024 * 1024):
    with Path(path).open('rb') as stream:
        data = stream.read(cap + 1)
    require(len(data) <= cap, 'JSON byte cap exceeded')
    def invalid(value):
        raise ValueError('nonfinite JSON token: ' + value)
    return json.loads(data, object_pairs_hook=_pairs, parse_constant=invalid)


def load_array(assets, manifest, name, code, dtype, columns, *, allow_empty=False):
    """Original probe signature, now with bounded shape/path validation."""
    entry = manifest['arrays'][name]
    shape = entry['shape']
    require(isinstance(shape, list) and len(shape) == 2, 'unexpected array shape: ' + name)
    require(isinstance(allow_empty, bool), 'invalid empty-array policy')
    rows = _int(shape[0], 0 if allow_empty else 1, 2**32 - 1, 'array row count')
    _int(shape[1], columns, columns, 'array column count')
    values = array(code)
    require(entry['dtype'] == dtype and values.itemsize == int(dtype[2:]),
            'unexpected array representation: ' + name)
    size = _int(entry['bytes'], 0 if allow_empty else 1, MAX_ARRAY_BYTES, 'array byte count')
    require(size == rows * columns * values.itemsize, 'array shape/size mismatch: ' + name)
    assets = Path(assets).resolve()
    relative = Path(entry['file'])
    require(not relative.is_absolute() and '..' not in relative.parts, 'array path escapes artifact')
    path = (assets / relative).resolve()
    require(path.is_relative_to(assets), 'array path escapes artifact')
    require(path.stat().st_size == size and file_hash(path) == entry['sha256'],
            'artifact checksum/size mismatch: ' + name)
    with path.open('rb') as stream:
        values.fromfile(stream, rows * columns)
        require(not stream.read(1), 'unexpected trailing array bytes: ' + name)
    if sys.byteorder != 'little':
        values.byteswap()
    return values


@dataclass(frozen=True)
class GeometryLimits:
    shells: int = 256
    nodes: int = 512

    def __post_init__(self):
        _int(self.shells, 1, 256, 'selected shell cap')
        _int(self.nodes, 1, 512, 'selected node cap')


@dataclass(frozen=True)
class SourceNode:
    source_id: int
    canonical_index: int
    position_m: tuple
    source_line: int
    codes: tuple
    blank_mask: int


@dataclass(frozen=True)
class SourceShell:
    source_id: int
    canonical_index: int
    source_line: int
    raw_record: tuple
    canonical_node_indices: tuple
    local_node_indices: tuple
    blank_mask: int
    arity: int


@dataclass(frozen=True)
class PartGeometry:
    part_id: int
    section_id: int
    material_id: int
    source_frame: str
    nodes: tuple
    shells: tuple
    canonical_manifest_sha256: str
    archive_sha256: str
    source_member: str
    member_sha256: str
    array_sha256: tuple


def _bits(values):
    return struct.pack('<' + 'd' * len(values), *values)


def verify_source_cards(archive_path, reference, nodes, shells, part_id):
    """Verify selected values and complete selected-shell coverage in one pass.

    Uses the original importer field widths/defaults. Other geometry bodies are
    streamed, not retained or admitted. This is not a new general deck parser.
    """
    require(Path(archive_path).stat().st_size <= MAX_SOURCE_BYTES, 'archive byte cap exceeded')
    require(file_hash(archive_path) == reference['archive_sha256'], 'archive SHA256 mismatch')
    member = reference['archive_member_prefix'] + VEHICLE
    expected_nodes = {n.source_id: n for n in nodes}
    expected_shells = {s.source_id: s for s in shells}
    seen_nodes, seen_shells = set(), set()
    digest = hashlib.sha256()
    size = 0
    keyword = None
    with zipfile.ZipFile(archive_path) as archive:
        require(archive.namelist().count(member) == 1, 'missing or duplicate vehicle member')
        require(archive.getinfo(member).file_size <= MAX_SOURCE_BYTES, 'source member byte cap exceeded')
        with archive.open(member) as stream:
            for number, raw in enumerate(iter(lambda: stream.readline(MAX_LINE_BYTES + 1), b''), 1):
                size += len(raw)
                require(len(raw) <= MAX_LINE_BYTES and size <= MAX_SOURCE_BYTES, 'source byte cap exceeded')
                digest.update(raw)
                text = raw.decode('ascii').split('$', 1)[0].rstrip('\r\n').rstrip()
                if text.lstrip().startswith('*'):
                    keyword = text.strip().upper()
                    continue
                if not text.strip():
                    continue
                if keyword == '*NODE':
                    nid = int(text[:8])
                    if nid not in expected_nodes:
                        continue
                    require(nid not in seen_nodes, 'duplicate selected source node')
                    seen_nodes.add(nid)
                    values, mask = fields(text, [8, 16, 16, 16, 8, 8],
                                          [int, float, float, float, int, int], [None, 0., 0., 0., 0, 0])
                    expected = expected_nodes[nid]
                    converted = tuple(v * .001 for v in values[1:4])
                    require(number == expected.source_line and mask == expected.blank_mask and
                            tuple(values[4:]) == expected.codes and _bits(converted) == _bits(expected.position_m),
                            'canonical node differs from pinned source card')
                elif keyword in ('*ELEMENT_SHELL', '*ELEMENT_SOLID', '*ELEMENT_BEAM'):
                    eid, pid = int(text[:8]), int(text[8:16])
                    if pid != part_id and eid not in expected_shells:
                        continue
                    require(keyword == '*ELEMENT_SHELL' and eid in expected_shells and eid not in seen_shells,
                            'missing/duplicate/unsupported selected source element')
                    seen_shells.add(eid)
                    values, mask = fields(text, [8] * 6, [int] * 6)
                    expected = expected_shells[eid]
                    require(number == expected.source_line and tuple(values) == expected.raw_record and
                            mask == expected.blank_mask, 'canonical shell differs from pinned source card')
    member_hash = digest.hexdigest()
    require(member_hash == reference['files'][VEHICLE]['sha256'], 'vehicle source SHA256 mismatch')
    require(seen_nodes == set(expected_nodes) and seen_shells == set(expected_shells),
            'selected source record coverage mismatch')
    return member, member_hash


def _load_part_geometry(asset_dir, archive_path, declarations, reference, limits):
    assets = Path(asset_dir)
    manifest_path = assets / 'manifest.json'
    m = read_json(manifest_path)
    require(m['schema'] == 'tlfea.yaris_source_vehicle_geometry.v1' and m['simulation_ready'] is False and
            m['canonical_geometry_validated'] is True and m['applied_rigid_transform'] is None and
            m['source_length_unit'] == 'mm' and m['output_length_unit'] == 'm' and m['length_scale'] == .001 and
            m['source_coordinate_frame'] == 'Untransformed original yaris-coarse-v1l.key coordinates',
            'expected original-frame SI canonical geometry')
    require(declarations.units.length_to_m == .001 and
            m['source_archive']['sha256'] == reference['archive_sha256'] and
            m['source_files'][VEHICLE]['sha256'] == reference['files'][VEHICLE]['sha256'],
            'canonical/source provenance mismatch')
    part = declarations.part
    matches = [p for p in m['parts'] if p['source_part_id'] == part.part_id]
    require(len(matches) == 1 and matches[0]['source_section_id'] == part.section_id and
            matches[0]['source_material_id'] == part.material_id and matches[0]['title'] == part.title,
            'canonical/declaration part join mismatch')
    specs = [('shells_records', 'Q', '<u8', 6), ('shells_node_indices', 'I', '<u4', 4),
             ('shells_source_lines', 'I', '<u4', 1), ('shells_blank_masks', 'H', '<u2', 1),
             ('node_ids', 'Q', '<u8', 1), ('node_positions', 'd', '<f8', 3),
             ('node_source_lines', 'I', '<u4', 1), ('node_codes', 'i', '<i4', 2),
             ('node_blank_masks', 'H', '<u2', 1)]
    require(sum(_int(m['arrays'][n]['bytes'], 1, MAX_ARRAY_BYTES, 'array bytes') for n, *_ in specs)
            <= MAX_ARRAY_BYTES, 'aggregate canonical array byte cap exceeded')
    arrays = {n: load_array(assets, m, n, code, dtype, cols) for n, code, dtype, cols in specs}
    ns = len(arrays['shells_source_lines'])
    nn = len(arrays['node_ids'])
    for name, _, _, width in specs:
        require(len(arrays[name]) == (ns if name.startswith('shells_') else nn) * width,
                'canonical parallel array size mismatch')
    selected, node_indices, eids = [], set(), set()
    records = arrays['shells_records']
    for row in range(ns):
        raw = tuple(records[6 * row:6 * row + 6])
        if raw[1] != part.part_id:
            continue
        require(len(selected) < limits.shells, 'selected shell cap exceeded')
        require(raw[0] > 0 and raw[0] not in eids, 'duplicate/invalid selected shell ID')
        eids.add(raw[0])
        arity = len(set(raw[2:]))
        require(arity == 4 or (arity == 3 and raw[4] == raw[5]), 'unsupported selected shell connectivity')
        indices = tuple(arrays['shells_node_indices'][4 * row:4 * row + 4])
        require(all(i < nn for i in indices), 'selected node index out of range')
        require(tuple(arrays['node_ids'][i] for i in indices) == raw[2:], 'node index/source ID disagreement')
        node_indices.update(indices)
        require(len(node_indices) <= limits.nodes, 'selected node cap exceeded')
        selected.append((row, raw, indices, arity))
    require(selected, 'part has no source shells')
    nodes, node_ids = [], set()
    for i in sorted(node_indices):
        nid = int(arrays['node_ids'][i])
        require(nid > 0 and nid not in node_ids, 'duplicate/invalid selected node ID')
        node_ids.add(nid)
        xyz = tuple(arrays['node_positions'][3 * i:3 * i + 3])
        require(all(math.isfinite(v) for v in xyz), 'nonfinite selected coordinate')
        nodes.append(SourceNode(nid, i, xyz, int(arrays['node_source_lines'][i]),
                                tuple(arrays['node_codes'][2 * i:2 * i + 2]), int(arrays['node_blank_masks'][i])))
    local = {n.canonical_index: i for i, n in enumerate(nodes)}
    shells = tuple(SourceShell(raw[0], row, int(arrays['shells_source_lines'][row]), raw, indices,
                               tuple(local[i] for i in indices), int(arrays['shells_blank_masks'][row]), arity)
                   for row, raw, indices, arity in selected)
    member, member_hash = verify_source_cards(archive_path, reference, nodes, shells, part.part_id)
    return PartGeometry(part.part_id, part.section_id, part.material_id, m['source_coordinate_frame'],
                        tuple(nodes), shells, file_hash(manifest_path), reference['archive_sha256'], member,
                        member_hash, tuple((name, m['arrays'][name]['sha256']) for name, *_ in specs))


def load_part_geometry(asset_dir, archive_path, declarations, reference, limits=GeometryLimits()):
    try:
        return _load_part_geometry(asset_dir, archive_path, declarations, reference, limits)
    except (KeyError, TypeError, IndexError, EOFError, RecursionError, OverflowError) as error:
        raise ValueError('malformed canonical/source geometry metadata') from error
