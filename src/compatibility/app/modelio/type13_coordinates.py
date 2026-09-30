"""Retain original working coordinates using the existing bounded source scanner."""
from dataclasses import dataclass

from ._legacy import VehicleGeometry, fields, require, scan_vehicle
from .canonical_geometry import _bits
from .canonical_incidence import _bounded_lines


@dataclass(frozen=True)
class WorkingNode:
    source_id: int
    source_line: int
    raw_text: str
    blank_mask: int
    position_native: tuple
    position_m: tuple
    codes: tuple


class _WorkingCollector(VehicleGeometry):
    def __init__(self, nodes, beams, part_id):
        super().__init__()
        self.expected_nodes = nodes
        self.expected_beams = beams
        self.part_id = part_id
        self.working_nodes = {}
        self.beam_rows = {}

    def metadata(self, keyword, records, number):
        pass  # A separate complete SourceIndex owns metadata.

    def record(self, keyword, line, number):
        if keyword == '*NODE':
            nid = fields(line[:8], [8], [int])[0][0]
            if nid not in self.expected_nodes:
                return
            values, mask = fields(line, [8, 16, 16, 16, 8, 8],
                                  [int, float, float, float, int, int], [None, 0., 0., 0., 0, 0])
            expected = self.expected_nodes[nid]
            si = tuple(v * .001 for v in values[1:4])
            require(nid not in self.working_nodes and number == expected['source_line'] and
                    mask == expected['blank_mask'] and tuple(values[4:]) == expected['codes'] and
                    _bits(si) == _bits(expected['position_m']), 'TYPE13 original NODE/canonical association changed')
            self.working_nodes[nid] = WorkingNode(nid, number, line, mask, tuple(values[1:4]), si, tuple(values[4:]))
            super().record(keyword, line, number)
        elif keyword == '*ELEMENT_BEAM':
            values, mask = fields(line, [8] * 10, [int] * 10, [None] * 4 + [0] * 6)
            if values[1] != self.part_id:
                return
            eid = values[0]
            require(eid in self.expected_beams and eid not in self.beam_rows,
                    'TYPE13 beam source coverage changed')
            expected = self.expected_beams[eid]
            require(tuple(values) == expected['raw_record'] and number == expected['source_line'] and
                    mask == expected['blank_mask'], 'TYPE13 original beam/canonical association changed')
            super().record(keyword, line, number)
            self.beam_rows[eid] = dict(expected, raw_text=line)


def collect_working_coordinates(stream, nodes, beams, part_id):
    require(0 < len(nodes) <= 8192 and 0 < len(beams) <= 8192,
            'TYPE13 working-coordinate extent exceeds selected startup limits')
    collector = _WorkingCollector(nodes, beams, part_id)
    summary = scan_vehicle(_bounded_lines(stream), 'yaris-coarse-v1l.key', collector)
    require(set(collector.working_nodes) == set(nodes) and set(collector.beam_rows) == set(beams),
            'TYPE13 original source has incomplete selected node/beam coverage')
    return collector.working_nodes, collector.beam_rows, summary
