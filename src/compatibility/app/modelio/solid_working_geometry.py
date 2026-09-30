"""Selected solid reference geometry, authenticated against original cards.

This reuses the canonical array reader and original keyword scanner. It retains
source slots and both coordinate representations; it admits no element law.
"""
import math
from pathlib import Path

from ._legacy import VehicleGeometry, fields, require, scan_vehicle
from .canonical_geometry import _bits, _int, load_array, read_json, file_hash, MAX_ARRAY_BYTES
from .canonical_incidence import _bounded_lines


def select_geometry(assets, part_ids, manifest_sha256, *, parent_cap=4096, node_cap=8192):
    assets = Path(assets)
    _int(parent_cap, 1, 4096, 'selected solid cap')
    _int(node_cap, 1, 8192, 'selected solid node cap')
    require(isinstance(part_ids, (list, tuple)) and 0 < len(part_ids) <= 32 and
            len(set(part_ids)) == len(part_ids), 'invalid selected solid parts')
    for pid in part_ids:
        _int(pid, 1, 2**63 - 1, 'selected part ID')
    manifest = read_json(assets / 'manifest.json')
    require(file_hash(assets / 'manifest.json') == manifest_sha256, 'canonical identity changed')
    require(manifest['schema'] == 'tlfea.yaris_source_vehicle_geometry.v1' and
            manifest['canonical_geometry_validated'] is True and manifest['simulation_ready'] is False and
            manifest['source_length_unit'] == 'mm' and manifest['output_length_unit'] == 'm' and
            manifest['length_scale'] == .001 and manifest['applied_rigid_transform'] is None,
            'unsupported source transform')
    wanted = frozenset(part_ids)
    specs = [('solids_records', 'Q', '<u8', 10), ('solids_node_indices', 'I', '<u4', 8),
             ('solids_source_lines', 'I', '<u4', 1), ('solids_blank_masks', 'H', '<u2', 1),
             ('node_ids', 'Q', '<u8', 1), ('node_positions', 'd', '<f8', 3),
             ('node_source_lines', 'I', '<u4', 1), ('node_blank_masks', 'H', '<u2', 1),
             ('node_codes', 'i', '<i4', 2)]
    require(sum(_int(manifest['arrays'][name]['bytes'], 1, MAX_ARRAY_BYTES, 'array bytes')
                for name, *_ in specs) <= MAX_ARRAY_BYTES, 'aggregate solid array byte cap exceeded')
    arrays = {name: load_array(assets, manifest, name, code, dtype, width)
              for name, code, dtype, width in specs}
    ns, nn = len(arrays['solids_source_lines']), len(arrays['node_ids'])
    for name, _, _, width in specs:
        require(len(arrays[name]) == (ns if name.startswith('solids_') else nn) * width,
                'parallel solid geometry array size mismatch')
    elements, indices = {}, set()
    records = arrays['solids_records']
    for i in range(len(records) // 10):
        row = tuple(records[10*i:10*i+10])
        if row[1] not in wanted:
            continue
        require(len(elements) < parent_cap and row[0] > 0 and row[0] not in elements and
                all(n > 0 for n in row[2:]), 'selected solid extent/identity')
        slots = tuple(arrays['solids_node_indices'][8*i:8*i+8])
        elements[row[0]] = dict(raw_record=row, canonical_index=i, canonical_node_indices=slots,
                               source_line=arrays['solids_source_lines'][i],
                               blank_mask=arrays['solids_blank_masks'][i])
        indices.update(slots)
        require(len(indices) <= node_cap, 'selected solid node cap exceeded')
    require(elements and 0 < len(indices) <= node_cap and
            {e['raw_record'][1] for e in elements.values()} == wanted, 'missing/oversized solid selection')
    nodes = {}
    for i in sorted(indices):
        require(i < len(arrays['node_ids']), 'solid canonical node index out of range')
        nid = arrays['node_ids'][i]
        require(nid > 0 and nid not in nodes, 'duplicate/invalid selected node identity')
        xyz = tuple(arrays['node_positions'][3*i:3*i+3])
        require(all(math.isfinite(v) for v in xyz), 'nonfinite selected coordinate')
        nodes[nid] = dict(canonical_index=i, source_line=arrays['node_source_lines'][i],
                         position_m=xyz,
                         blank_mask=arrays['node_blank_masks'][i],
                         codes=tuple(arrays['node_codes'][2*i:2*i+2]))
    for element in elements.values():
        require(tuple(arrays['node_ids'][i] for i in element['canonical_node_indices']) ==
                element['raw_record'][2:], 'solid record/index association changed')
    return manifest, nodes, elements, {name: manifest['arrays'][name] for name, *_ in specs}


class _Collector(VehicleGeometry):
    def __init__(self, nodes, elements, part_ids):
        super().__init__()
        self.expected_nodes, self.expected_elements = nodes, elements
        self.part_ids = frozenset(part_ids)
        self.working_nodes, self.solid_rows = {}, {}

    def metadata(self, keyword, records, number):
        pass  # The complete SourceIndex authenticates declarations separately.

    def record(self, keyword, line, number):
        if keyword == '*NODE':
            nid = fields(line[:8], [8], [int])[0][0]
            if nid not in self.expected_nodes:
                return
            values, mask = fields(line, [8, 16, 16, 16, 8, 8],
                                  [int, float, float, float, int, int], [None, 0., 0., 0., 0, 0])
            expected = self.expected_nodes[nid]
            si = tuple(x * .001 for x in values[1:4])
            require(nid not in self.working_nodes and number == expected['source_line'] and
                    mask == expected['blank_mask'] and tuple(values[4:]) == expected['codes'] and
                    _bits(si) == _bits(expected['position_m']), 'solid original NODE/canonical mismatch')
            self.working_nodes[nid] = dict(expected, position_native=tuple(values[1:4]), raw_text=line)
        elif keyword in ('*ELEMENT_SOLID', '*ELEMENT_SHELL', '*ELEMENT_BEAM'):
            eid, pid = fields(line[:16], [8, 8], [int, int])[0]
            if pid not in self.part_ids and eid not in self.expected_elements:
                return
            require(keyword == '*ELEMENT_SOLID' and eid in self.expected_elements and
                    eid not in self.solid_rows, 'selected solid source coverage changed')
            values, mask = fields(line, [8]*10, [int]*10)
            expected = self.expected_elements[eid]
            require(tuple(values) == expected['raw_record'] and number == expected['source_line'] and
                    mask == expected['blank_mask'], 'solid original element/canonical mismatch')
            self.solid_rows[eid] = dict(expected, raw_text=line)


def collect_working_solids(stream, nodes, elements, part_ids):
    require(0 < len(nodes) <= 8192 and 0 < len(elements) <= 4096, 'solid working extent exceeded')
    collector = _Collector(nodes, elements, part_ids)
    summary = scan_vehicle(_bounded_lines(stream), 'yaris-coarse-v1l.key', collector)
    require(set(collector.working_nodes) == set(nodes) and set(collector.solid_rows) == set(elements),
            'incomplete selected solid source coverage')
    return collector.working_nodes, collector.solid_rows, summary
