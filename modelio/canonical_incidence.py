"""Bounded one-hop canonical incidence, checked against the original collector.

All incident raw shell/solid/beam records are retained; beam orientation fields
are preserved but are not treated as physical endpoint connectivity. No neighbor
geometry expansion, coordinate transform, mechanics or transitive closure.
"""
from dataclasses import dataclass
import math
from pathlib import Path
import zipfile

from ._legacy import fields, file_sha256, require, VehicleGeometry, scan_vehicle, FIELDS
from .canonical_geometry import load_array, read_json, SourceNode, _bits, MAX_ARRAY_BYTES
from .source_blocks import MAX_LINE_BYTES, MAX_SOURCE_BYTES


@dataclass(frozen=True)
class IncidentElement:
    family: str
    source_id: int
    part_id: int
    canonical_index: int
    source_line: int
    raw_record: tuple
    canonical_node_indices: tuple
    touched_node_ids: tuple
    blank_mask: int


@dataclass(frozen=True)
class CanonicalIncidence:
    nodes: tuple
    elements: tuple
    array_sha256: tuple
    source_member_sha256: str
    source_records_verified: bool


class _Subset(VehicleGeometry):
    """Keep the owning importer's field/default/uniqueness arithmetic unchanged."""
    def __init__(self, node_ids, element_cap):
        super().__init__()
        self.wanted = frozenset(node_ids)
        self.element_cap = element_cap

    def metadata(self, keyword, records, number):
        pass  # Caller owns the separate typed metadata index.

    def record(self, keyword, line, number):
        if keyword == '*NODE':
            if int(line[:8]) not in self.wanted:
                return
        else:
            stop = 4 if keyword == '*ELEMENT_BEAM' else 6 if keyword == '*ELEMENT_SHELL' else 10
            connected = fields(line[16:8*stop], [8] * (stop-2), [int] * (stop-2))[0]
            if not self.wanted.intersection(connected):
                return
            require(len(self.element_ids) < self.element_cap, 'source incident element cap exceeded')
        super().record(keyword, line, number)


def _bounded_lines(stream):
    size = keywords = 0
    while True:
        raw = stream.readline(MAX_LINE_BYTES + 1)
        if not raw:
            return
        size += len(raw)
        keywords += raw.lstrip().startswith(b'*')
        require(size <= MAX_SOURCE_BYTES and len(raw) <= MAX_LINE_BYTES and keywords <= 10000,
                'incident source byte/keyword cap exceeded')
        yield raw


def _load_incidence(asset_dir, archive_path, reference, geometry, node_ids, element_cap):
    require(isinstance(element_cap, int) and not isinstance(element_cap, bool) and 1 <= element_cap <= 4096,
            'invalid incident element cap')
    require(len(node_ids) <= 512 and len(set(node_ids)) == len(node_ids) and
            all(isinstance(n, int) and not isinstance(n, bool) and 0 < n < 2**63 for n in node_ids),
            'invalid selected/external node union')
    wanted = frozenset(node_ids)
    require(wanted and {n.source_id for n in geometry.nodes} <= wanted, 'missing selected geometry nodes')
    assets = Path(asset_dir)
    m = read_json(assets / 'manifest.json')
    require(file_sha256(assets / 'manifest.json') == geometry.canonical_manifest_sha256 and
            geometry.archive_sha256 == reference['archive_sha256'] and
            geometry.member_sha256 == reference['files']['yaris-coarse-v1l.key']['sha256'],
            'incidence geometry/source provenance mismatch')
    specs = [('node_ids', 'Q', '<u8', 1), ('node_positions', 'd', '<f8', 3),
             ('node_source_lines', 'I', '<u4', 1), ('node_codes', 'i', '<i4', 2),
             ('node_blank_masks', 'H', '<u2', 1)]
    for family in FIELDS:
        specs += [(family+'_records', 'Q', '<u8', len(FIELDS[family])),
                  (family+'_node_indices', 'I', '<u4', 2 if family == 'beams' else len(FIELDS[family])-2),
                  (family+'_source_lines', 'I', '<u4', 1), (family+'_blank_masks', 'H', '<u2', 1)]
    sizes = [m['arrays'][n]['bytes'] for n, *_ in specs]
    require(all(type(s) is int and 0 <= s <= MAX_ARRAY_BYTES for s in sizes) and
            sum(sizes) <= MAX_ARRAY_BYTES, 'aggregate incidence array byte cap exceeded')
    a = {n: load_array(assets, m, n, code, dtype, width, allow_empty=not n.startswith('node_'))
         for n, code, dtype, width in specs}
    nn = len(a['node_ids'])
    for n, _, _, width in specs[:5]:
        require(len(a[n]) == nn*width, 'parallel incidence node array size mismatch')
    nodes = []
    for i, nid in enumerate(a['node_ids']):
        if nid in wanted:
            xyz = tuple(a['node_positions'][3*i:3*i+3])
            require(all(math.isfinite(x) for x in xyz), 'nonfinite incident node')
            nodes.append(SourceNode(nid, i, xyz, a['node_source_lines'][i],
                                    tuple(a['node_codes'][2*i:2*i+2]), a['node_blank_masks'][i]))
    require(len(nodes) == len(wanted) and {n.source_id for n in nodes} == wanted,
            'missing/duplicate canonical incident node')
    elements, seen = [], set()
    for family in FIELDS:
        width = len(FIELDS[family]); nc = 2 if family == 'beams' else width-2
        records = a[family+'_records']; lines = a[family+'_source_lines']
        require(len(records) == len(lines)*width and len(a[family+'_node_indices']) == len(lines)*nc and
                len(a[family+'_blank_masks']) == len(lines), 'parallel incidence element array size mismatch')
        for i in range(len(lines)):
            raw = tuple(records[i*width:(i+1)*width])
            connected = raw[2:2+nc]
            touched = tuple(sorted(wanted.intersection(connected)))
            if not touched:
                continue
            require(len(elements) < element_cap, 'canonical incident element cap exceeded')
            require(raw[0] > 0 and raw[1] > 0 and raw[0] not in seen, 'duplicate/invalid incident element ID')
            seen.add(raw[0])
            indices = tuple(a[family+'_node_indices'][i*nc:(i+1)*nc])
            require(all(j < nn for j in indices) and tuple(a['node_ids'][j] for j in indices) == connected,
                    'incident node index/source ID disagreement')
            elements.append(IncidentElement(family, raw[0], raw[1], i, lines[i], raw, indices, touched,
                                            a[family+'_blank_masks'][i]))
    require(Path(archive_path).stat().st_size <= MAX_SOURCE_BYTES and
            file_sha256(archive_path) == reference['archive_sha256'], 'incident archive SHA/byte mismatch')
    subset = _Subset(wanted, element_cap)
    member = reference['archive_member_prefix'] + 'yaris-coarse-v1l.key'
    with zipfile.ZipFile(archive_path) as archive:
        require(archive.namelist().count(member) == 1 and archive.getinfo(member).file_size <= MAX_SOURCE_BYTES,
                'missing/duplicate/oversized incident source member')
        with archive.open(member) as stream:
            source = scan_vehicle(_bounded_lines(stream), 'yaris-coarse-v1l.key', subset)
    require(source['sha256'] == geometry.member_sha256, 'incident source member SHA mismatch')
    require(set(subset.node_ids) == wanted and len(subset.node_ids) == len(nodes), 'raw incident node coverage mismatch')
    for n in nodes:
        j = subset.node_index[n.source_id]
        require(n.source_line == subset.node_lines[j] and n.blank_mask == subset.node_masks[j] and
                n.codes == tuple(subset.node_codes[2*j:2*j+2]) and
                _bits(n.position_m) == _bits(subset.positions[3*j:3*j+3]), 'canonical incident node differs from source')
    expected = {(e.family, e.source_id): e for e in elements}
    actual_count = 0
    for family, records in subset.elements.items():
        width = len(FIELDS[family])
        for j in range(len(records)//width):
            actual_count += 1
            raw = tuple(records[j*width:(j+1)*width]); key = (family, raw[0])
            require(key in expected, 'canonical incidence omitted a source element')
            e = expected[key]
            require(raw == e.raw_record and subset.element_lines[family][j] == e.source_line and
                    subset.element_masks[family][j] == e.blank_mask, 'canonical incident element differs from source')
    require(actual_count == len(expected), 'extra canonical incident element')
    return CanonicalIncidence(tuple(nodes), tuple(elements),
                              tuple((n, m['arrays'][n]['sha256']) for n, *_ in specs), source['sha256'], True)


def load_incidence(asset_dir, archive_path, reference, geometry, node_ids, element_cap=4096):
    """Requires a source-verified PartGeometry; rechecks all incident source data."""
    try:
        return _load_incidence(asset_dir, archive_path, reference, geometry, tuple(node_ids), element_cap)
    except (KeyError, TypeError, IndexError, EOFError, OverflowError) as error:
        raise ValueError('malformed canonical incidence metadata') from error
