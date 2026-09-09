#!/usr/bin/env python3
"""Compile exact original Yaris geometry, with a blocking mechanics inventory.

Streaming, offline, standard-library only. No solver, GPU, whole-case include
assembly, or material/constraint interpretation is performed. Binary arrays
retain source order and repeated connectivity slots exactly.
"""
import argparse
from array import array
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import zipfile

from import_yaris_wall import MM_TO_M, WallImportError, require, sha256

REFERENCE = Path(__file__).resolve().parents[1] / 'models/yaris_coarse_v1l.json'
VEHICLE = 'yaris-coarse-v1l.key'
GEOMETRY_KEYWORDS = {'*NODE', '*ELEMENT_SHELL', '*ELEMENT_SOLID', '*ELEMENT_BEAM'}
FIELDS = {
    'shells': ['element_id', 'part_id', 'n1', 'n2', 'n3', 'n4'],
    'solids': ['element_id', 'part_id', 'n1', 'n2', 'n3', 'n4', 'n5', 'n6', 'n7', 'n8'],
    'beams': ['element_id', 'part_id', 'n1', 'n2', 'n3', 'rt1', 'rr1', 'rt2', 'rr2', 'local'],
}


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def fields(line, widths, types, defaults=None):
    """Fixed fields, with explicit zero placeholders AND a missing-field mask."""
    values, mask, offset = [], 0, 0
    for index, (width, convert) in enumerate(zip(widths, types)):
        text = line[offset:offset + width].strip()
        offset += width
        if not text:
            require(defaults is not None and defaults[index] is not None,
                    f'missing required field {index + 1}')
            mask |= 1 << index
            value = defaults[index]
        else:
            try:
                value = convert(text.replace('D', 'E').replace('d', 'e'))
            except ValueError as error:
                raise WallImportError(f'invalid field {index + 1}: {text}') from error
            require(not isinstance(value, float) or math.isfinite(value), 'nonfinite geometry field')
        values.append(value)
    require(not line[offset:].strip(), 'unsupported extra geometry fields')
    return values, mask


class VehicleGeometry:
    def __init__(self):
        self.node_ids = array('Q')
        self.positions = array('d')
        self.node_codes = array('i')
        self.node_masks = array('H')
        self.node_lines = array('I')
        self.node_index = {}
        self.elements = {name: array('Q') for name in FIELDS}
        self.element_masks = {name: array('H') for name in FIELDS}
        self.element_lines = {name: array('I') for name in FIELDS}
        self.element_ids = set()
        self.parts, self.sections, self.materials = {}, {}, {}
        self.connectivity = {}
        self.blank_coordinate_nodes = []

    def record(self, keyword, line, number):
        if keyword == '*NODE':
            values, mask = fields(line, [8, 16, 16, 16, 8, 8],
                                  [int, float, float, float, int, int],
                                  [None, 0.0, 0.0, 0.0, 0, 0])
            nid, x, y, z, tc, rc = values
            require(0 < nid <= 2**64 - 1 and nid not in self.node_index,
                    f'duplicate/invalid node ID {nid}')
            require(-2**31 <= tc < 2**31 and -2**31 <= rc < 2**31, 'node code overflow')
            self.node_index[nid] = len(self.node_ids)
            self.node_ids.append(nid)
            self.positions.extend((x * MM_TO_M, y * MM_TO_M, z * MM_TO_M))
            self.node_codes.extend((tc, rc))
            self.node_masks.append(mask)
            self.node_lines.append(number)
            if mask & 14:
                self.blank_coordinate_nodes.append(dict(source_node_id=nid, source_line=number,
                                                        xyz_blank_mask=(mask >> 1) & 7))
            return
        name = {'*ELEMENT_SHELL': 'shells', '*ELEMENT_SOLID': 'solids', '*ELEMENT_BEAM': 'beams'}[keyword]
        count = len(FIELDS[name])
        defaults = [None] * count
        if name == 'beams':
            defaults[4:] = [0] * 6
        values, mask = fields(line, [8] * count, [int] * count, defaults)
        eid, pid = values[:2]
        require(0 < eid <= 2**64 - 1 and eid not in self.element_ids,
                f'duplicate/invalid structural element ID {eid}')
        require(pid > 0 and all(0 <= v <= 2**64 - 1 for v in values), 'negative/out-of-range source field')
        required_nodes = values[2:4] if name == 'beams' else values[2:]
        require(all(n > 0 for n in required_nodes), 'missing structural node ID')
        unique = len(set(required_nodes))
        require(unique in ({3, 4} if name == 'shells' else {6, 8} if name == 'solids' else {1, 2}),
                f'unsupported {name} connectivity pattern')
        # Equal-position/zero-length beam endpoints are retained; spotweld and
        # beam formulation semantics are explicitly not inferred here.
        self.element_ids.add(eid)
        self.elements[name].extend(values)
        self.element_masks[name].append(mask)
        self.element_lines[name].append(number)

    def metadata(self, keyword, records, number):
        if keyword == '*PART':
            require(len(records) == 2, '*PART must have title and identity card')
            vals, mask = fields(records[1], [10] * 8, [int] * 8,
                                [None, None, None, 0, 0, 0, 0, 0])
            pid, sid, mid = vals[:3]
            require(pid > 0 and pid not in self.parts, 'duplicate/invalid part ID')
            self.parts[pid] = dict(source_part_id=pid, title=records[0].strip(), source_section_id=sid,
                                   source_material_id=mid, source_line=number,
                                   raw_fields=vals, blank_field_mask=mask)
        elif keyword.startswith('*SECTION_'):
            require(records, 'empty section block')
            sid = fields(records[0][:10], [10], [int])[0][0]
            require(sid > 0 and sid not in self.sections, 'duplicate/invalid section ID')
            self.sections[sid] = dict(source_section_id=sid, keyword=keyword,
                                      formulation_field_raw=records[0][10:20].strip(), source_line=number,
                                      mechanics_status='BLOCKING')
        elif keyword.startswith('*MAT_'):
            require(records, 'empty material block')
            mid = fields(records[0][:10], [10], [int])[0][0]
            require(mid > 0 and mid not in self.materials, 'duplicate/invalid material ID')
            self.materials[mid] = dict(source_material_id=mid, keyword=keyword,
                                       source_line=number, mechanics_status='BLOCKING')

    def validate(self):
        require(self.node_ids and any(self.elements.values()), 'missing vehicle geometry')
        require(len(self.node_ids) < 2**32, 'node count exceeds compact index format')
        for part in self.parts.values():
            require(part['source_section_id'] in self.sections, 'unresolved part section reference')
            require(part['source_material_id'] in self.materials, 'unresolved part material reference')
        patterns = {}
        for name, records in self.elements.items():
            width = len(FIELDS[name])
            indices = array('I')
            pattern = Counter()
            for offset in range(0, len(records), width):
                row = records[offset:offset + width]
                require(row[1] in self.parts, f'{name} {row[0]}: unresolved part ID {row[1]}')
                nodes = row[2:4] if name == 'beams' else row[2:]
                pattern[len(set(nodes))] += 1
                for nid in nodes:
                    require(nid in self.node_index, f'{name} {row[0]}: missing node {nid}')
                    indices.append(self.node_index[nid])
            self.connectivity[name] = indices
            patterns[name] = {str(k): v for k, v in sorted(pattern.items())}
        return patterns


def disposition(keyword, geometry_scope):
    if keyword in ('*KEYWORD', '*END', '*TITLE'):
        return 'METADATA_ONLY', 'Source structure/title only; no solver behavior'
    if geometry_scope and keyword in GEOMETRY_KEYWORDS:
        return 'GEOMETRY_ONLY', 'Coordinates/raw connectivity only; element mechanics and flags are not executed'
    if geometry_scope and keyword == '*PART':
        return 'REFERENCES_ONLY', 'Part title and section/material IDs only; no mass or dynamics'
    return 'BLOCKING', 'Not implemented by this geometry compiler; original source block retained in archive'


def scan(stream, filename, geometry=None):
    """One bounded-memory pass, retaining provenance for every keyword block."""
    ledger, census = [], Counter()
    current = None
    capture = []
    source_hash = hashlib.sha256()
    ended = False
    for number, raw in enumerate(stream, 1):
        source_hash.update(raw)
        require(number < 2**32, 'source line number overflow')
        # Comments are kept in block hashes, but never treated as active cards.
        stripped = raw.lstrip()
        is_comment = stripped.startswith(b'$')
        line = '' if is_comment else raw.decode('ascii').split('$', 1)[0].rstrip()
        if line.lstrip().startswith('*'):
            require(not ended, f'{filename}:{number}: keyword after *END')
            if current is not None:
                finish_block(current, capture, ledger, geometry)
            keyword = line.strip().upper()
            require(current is not None or keyword == '*KEYWORD', f'{filename}: missing *KEYWORD')
            status, reason = disposition(keyword, geometry is not None)
            current = dict(keyword=keyword, file=filename, first_line=number, last_line=number,
                           data_records=0, disposition=status, reason=reason, digest=hashlib.sha256())
            capture = []
            census[keyword] += 1
            ended = keyword == '*END'
        elif line.strip():
            require(current is not None and not ended, f'{filename}:{number}: data outside active block')
            current['data_records'] += 1
            keyword = current['keyword']
            try:
                if geometry is not None and keyword in GEOMETRY_KEYWORDS:
                    geometry.record(keyword, line, number)
                elif geometry is not None and (keyword == '*PART' or
                                               keyword.startswith(('*SECTION_', '*MAT_'))):
                    # Full physics cards remain in the archive, not an invented
                    # interpretation. Only identity/form references are read.
                    require(len(capture) < 1024, 'unexpected metadata block length')
                    capture.append(line)
            except (WallImportError, OverflowError) as error:
                raise WallImportError(f'{filename}:{number}: {error}') from error
        if current is not None:
            current['digest'].update(raw)
            current['last_line'] = number
    require(current is not None and ended, f'{filename}: missing *END')
    finish_block(current, capture, ledger, geometry)
    return dict(sha256=source_hash.hexdigest(), keyword_counts=dict(sorted(census.items())), blocks=ledger)


def finish_block(block, capture, ledger, geometry):
    if geometry is not None:
        geometry.metadata(block['keyword'], capture, block['first_line'])
    block['source_block_sha256'] = block.pop('digest').hexdigest()
    ledger.append(block)


def assert_simulation_ready(manifest):
    require(manifest.get('simulation_ready') is True, 'simulation admission blocked: geometry-only model')
    require(not any(b['disposition'] == 'BLOCKING' for f in manifest['source_files'].values()
                    for b in f['blocks']), 'simulation admission blocked: unresolved keyword mechanics')


def write_array(path, values, columns, field_names=None):
    require(values.itemsize in (2, 4, 8) and len(values) % columns == 0, 'invalid array layout')
    dtype = {'Q': '<u8', 'I': '<u4', 'H': '<u2', 'i': '<i4', 'd': '<f8'}[values.typecode]
    if sys.byteorder != 'little':
        values = array(values.typecode, values)
        values.byteswap()
    with path.open('wb') as output:
        values.tofile(output)
    return dict(dtype=dtype, shape=[len(values) // columns, columns],
                bytes=path.stat().st_size, sha256=file_sha256(path), fields=field_names)


def compile_archive(archive_path, output, require_simulation=False):
    reference = json.loads(REFERENCE.read_text())
    require(file_sha256(archive_path) == reference['archive_sha256'], 'archive SHA256 mismatch')
    geometry, source_files = VehicleGeometry(), {}
    with zipfile.ZipFile(archive_path) as archive:
        for name in [VEHICLE] + sorted(n for n in reference['files'] if n != VEHICLE):
            member = reference['archive_member_prefix'] + name
            require(archive.namelist().count(member) == 1, f'missing/duplicate archive member {member}')
            info = archive.getinfo(member)
            require(info.file_size <= 64 * 1024 * 1024, 'source member exceeds bounded import size')
            with archive.open(member) as stream:
                summary = scan(stream, name, geometry if name == VEHICLE else None)
            require(summary['sha256'] == reference['files'][name]['sha256'], f'{name}: source SHA256 mismatch')
            require(summary['keyword_counts'] == reference['files'][name]['keyword_counts'],
                    f'{name}: keyword census differs from inspected model')
            source_files[name] = summary
    patterns = geometry.validate()
    counts = dict(nodes=len(geometry.node_ids), parts=len(geometry.parts),
                  **{name: len(rows) // len(FIELDS[name]) for name, rows in geometry.elements.items()})
    require(counts == reference['expected_vehicle_geometry'], 'vehicle geometry counts differ from reference')
    for name, expected in reference['expected_unique_node_patterns'].items():
        require(patterns[name] == expected, f'{name}: unique-node patterns differ from reference')
    manifest = dict(schema='tlfea.yaris_source_vehicle_geometry.v1', model=reference['model'],
                    scope='Original vehicle include only; setup/wall/whole-case assembly and all physics remain blocked',
                    simulation_ready=False, m1_complete=False, canonical_geometry_validated=True,
                    source_coordinate_frame='Untransformed original yaris-coarse-v1l.key coordinates',
                    source_length_unit='mm', output_length_unit='m', length_scale=MM_TO_M,
                    applied_rigid_transform=None, source_archive=dict(file='source_model.zip', sha256=reference['archive_sha256']),
                    source_files=source_files, counts=counts, unique_node_patterns=patterns,
                    blank_coordinate_mapping=dict(rule='Explicit zero coordinate for source blank fields',
                                                  affected_nodes=geometry.blank_coordinate_nodes),
                    beam_policy='All raw slots preserved; only n1/n2 become endpoint indices; n3/local/release semantics blocked',
                    repeated_connectivity_policy='Preserve source node repetitions and order; no shell/solid formulation inferred',
                    node_constraint_policy='TC/RC fields and blank mask preserved, not enforced',
                    parts=list(geometry.parts.values()), sections=list(geometry.sections.values()),
                    materials=list(geometry.materials.values()),
                    generator=dict(file='tools/import_yaris_vehicle.py', sha256=sha256(Path(__file__).read_bytes()),
                                   shared_wall_utilities_sha256=sha256(Path(__file__).with_name('import_yaris_wall.py').read_bytes()),
                                   model_reference_sha256=sha256(REFERENCE.read_bytes())),
                    arrays={})
    if require_simulation:
        assert_simulation_ready(manifest)
    # Nothing is published until all source/identity/connectivity gates pass.
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    array_dir = output / 'arrays'
    array_dir.mkdir(exist_ok=True)
    arrays = {'node_ids': (geometry.node_ids, 1, ['source_node_id']),
              'node_positions': (geometry.positions, 3, ['x_m', 'y_m', 'z_m']),
              'node_codes': (geometry.node_codes, 2, ['tc_raw', 'rc_raw']),
              'node_blank_masks': (geometry.node_masks, 1, ['blank_field_mask']),
              'node_source_lines': (geometry.node_lines, 1, ['source_line'])}
    for name in FIELDS:
        arrays[name + '_records'] = (geometry.elements[name], len(FIELDS[name]), FIELDS[name])
        arrays[name + '_source_lines'] = (geometry.element_lines[name], 1, ['source_line'])
        arrays[name + '_blank_masks'] = (geometry.element_masks[name], 1, ['blank_field_mask'])
        arrays[name + '_node_indices'] = (geometry.connectivity[name], 2 if name == 'beams' else len(FIELDS[name]) - 2, None)
    for name, (values, columns, names) in arrays.items():
        relative = 'arrays/' + name + '.bin'
        manifest['arrays'][name] = dict(file=relative, **write_array(output / relative, values, columns, names))
    # Retain the original compressed archive outside git for reproducible later
    # interpretation of every currently blocking mechanics card.
    archive_copy = output / 'source_model.zip'
    if Path(archive_path).resolve() != archive_copy.resolve():
        shutil.copyfile(archive_path, archive_copy)
    require(file_sha256(archive_copy) == reference['archive_sha256'], 'staged archive verification failed')
    manifest_bytes = (json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + '\n').encode()
    temporary = output / 'manifest.json.tmp'
    temporary.write_bytes(manifest_bytes)
    temporary.replace(output / 'manifest.json')
    sums = {a['file']: a['sha256'] for a in manifest['arrays'].values()}
    sums.update({'source_model.zip': reference['archive_sha256'], 'manifest.json': sha256(manifest_bytes)})
    (output / 'SHA256SUMS').write_text(''.join(f'{value}  {name}\n' for name, value in sorted(sums.items())))
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-archive', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--require-simulation-ready', action='store_true',
                        help='Admission gate: fails because this importer does not implement mechanics')
    args = parser.parse_args()
    try:
        manifest = compile_archive(args.source_archive, args.output, args.require_simulation_ready)
    except (OSError, ValueError, zipfile.BadZipFile) as error:
        parser.exit(1, f'vehicle geometry import failed: {error}\n')
    print(json.dumps(dict(output=str(args.output), counts=manifest['counts'],
                          simulation_ready=manifest['simulation_ready'], m1_complete=False), sort_keys=True))


if __name__ == '__main__':
    main()
