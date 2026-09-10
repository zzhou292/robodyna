"""Authenticate complete canonical geometry with the original streaming importer."""
from dataclasses import dataclass
from pathlib import Path
import zipfile

from ._legacy import VehicleGeometry, FIELDS, scan_vehicle, file_sha256, require
from .canonical_geometry import load_array, read_json, MAX_ARRAY_BYTES
from .canonical_incidence import _bounded_lines
from .source_blocks import scan_metadata, DECLARATION_FAMILIES, ATTACHMENT_FAMILIES, MAX_SOURCE_BYTES
from .spotweld_cards import scan_spotwelds
from .yaris_part import _compile_archive_part, _member, REFERENCE, VEHICLE


@dataclass
class FullShellSource:
    geometry: object
    index: object
    welds: object
    units: object
    authority: dict
    manifest: dict
    manifest_sha256: str
    source_files: dict


def verify_canonical(assets, manifest, geometry, source_files, reference):
    require(manifest['schema'] == 'tlfea.yaris_source_vehicle_geometry.v1' and
            manifest['canonical_geometry_validated'] is True and
            manifest['applied_rigid_transform'] is None and manifest['length_scale'] == .001 and
            manifest['output_length_unit'] == 'm' and
            manifest['source_archive']['sha256'] == reference['archive_sha256'],
            'full-shell scope requires original untransformed SI canonical geometry')
    counts = dict(nodes=len(geometry.node_ids), parts=len(geometry.parts),
                  **{n: len(v)//len(FIELDS[n]) for n, v in geometry.elements.items()})
    require(counts == manifest['counts'] == reference['expected_vehicle_geometry'],
            'complete canonical/source entity coverage differs')
    require(manifest['source_files'] == source_files, 'canonical keyword/source-block inventory differs')
    for name in ('parts', 'materials', 'sections'):
        table = getattr(geometry, name)
        require(manifest[name] == [table[k] for k in sorted(table)], 'canonical ' + name + ' table differs')
    specs = [('node_ids', geometry.node_ids, '<u8', 1),
             ('node_positions', geometry.positions, '<f8', 3),
             ('node_codes', geometry.node_codes, '<i4', 2),
             ('node_blank_masks', geometry.node_masks, '<u2', 1),
             ('node_source_lines', geometry.node_lines, '<u4', 1)]
    for family in FIELDS:
        specs += [(family+'_records', geometry.elements[family], '<u8', len(FIELDS[family])),
                  (family+'_node_indices', geometry.connectivity[family], '<u4', 2 if family == 'beams' else len(FIELDS[family])-2),
                  (family+'_source_lines', geometry.element_lines[family], '<u4', 1),
                  (family+'_blank_masks', geometry.element_masks[family], '<u2', 1)]
    require(set(manifest['arrays']) == {s[0] for s in specs}, 'unexpected canonical array inventory')
    require(sum(manifest['arrays'][s[0]]['bytes'] for s in specs) <= MAX_ARRAY_BYTES,
            'aggregate canonical array byte cap exceeded')
    for name, expected, dtype, width in specs:
        actual = load_array(assets, manifest, name, expected.typecode, dtype, width, allow_empty=True)
        require(actual.tobytes() == expected.tobytes(), 'canonical array differs from original source: ' + name)


def load_full_shell_source(archive_path, assets):
    # Reuse the established pinned archive/README/unit authentication. This
    # anchor selects no physics and does not change the small-part compiler.
    seed, authority = _compile_archive_part(archive_path, 2000157)
    reference = read_json(REFERENCE)
    manifest_path = Path(assets)/'manifest.json'
    manifest = read_json(manifest_path)
    geometry, source_files = VehicleGeometry(), {}
    with zipfile.ZipFile(archive_path) as archive:
        for name in [VEHICLE] + sorted(n for n in reference['files'] if n != VEHICLE):
            member = reference['archive_member_prefix'] + name
            _member(archive, member, MAX_SOURCE_BYTES)
            with archive.open(member) as stream:
                summary = scan_vehicle(_bounded_lines(stream), name, geometry if name == VEHICLE else None)
            require(summary['sha256'] == reference['files'][name]['sha256'] and
                    summary['keyword_counts'] == reference['files'][name]['keyword_counts'],
                    'full-shell source member identity/census differs: ' + name)
            source_files[name] = summary
        member = reference['archive_member_prefix'] + VEHICLE
        with archive.open(member) as stream:
            index = scan_metadata(stream, VEHICLE, families=DECLARATION_FAMILIES | ATTACHMENT_FAMILIES)
        with archive.open(member) as stream:
            welds = scan_spotwelds(stream, VEHICLE)
    require(index.sha256 == welds.source_sha256 == reference['files'][VEHICLE]['sha256'],
            'full-shell metadata/source identity differs')
    require(all(len(v) == 1 for v in index.entries.values()), 'duplicate full-shell metadata identity')
    patterns = geometry.validate()
    require(all(patterns[k] == v for k, v in reference['expected_unique_node_patterns'].items()),
            'complete source topology pattern differs')
    verify_canonical(assets, manifest, geometry, source_files, reference)
    return FullShellSource(geometry, index, welds, seed.units, authority['source'], manifest,
                           file_sha256(manifest_path), source_files)
