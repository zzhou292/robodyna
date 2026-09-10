"""Whole-part declaration inventory for the explicit QEPH/C0 donor policy.

The original single-part compiler remains byte-for-byte unchanged. Admitting
ELFORM16 here records a converter policy, not LS-DYNA formulation equivalence.
"""
from dataclasses import dataclass

from ._legacy import require
from .declarations import PartDeclarations, SectionShell
from .keyword_cards import (card, parse_part, parse_material, parse_curve,
                            SECTION1, SECTION2)


@dataclass(frozen=True)
class AssemblyLimits:
    parts: int = 8
    shells: int = 1024
    nodes: int = 2048
    groups: int = 64
    spotwelds: int = 128
    frontier_nodes: int = 256
    incident_elements: int = 4096

    def __post_init__(self):
        maxima = dict(parts=8, shells=1024, nodes=2048, groups=64,
                      spotwelds=128, frontier_nodes=256, incident_elements=4096)
        for name, maximum in maxima.items():
            value = getattr(self, name)
            require(type(value) is int and 0 < value <= maximum,
                    'invalid assembly ' + name + ' cap')


def assembly_section(block, units):
    require(block.keyword == '*SECTION_SHELL' and len(block.cards) == 2,
            'assembly requires two-card SECTION_SHELL declaration')
    first = card(block.cards[0], SECTION1, [int, int, float, int, float, float, int, int])
    second = card(block.cards[1], SECTION2, [float] * 6 + [int, int])
    require(first.get('secid') is not None and first.get('secid') > 0 and
            first.get('elform') in (2, 16) and first.get('nip') == 3,
            'assembly donor policy requires explicit ELFORM2/16 and NIP3')
    require(all(first.get(n) is None for n in ('shrf', 'propt', 'qr_irid', 'icomp', 'setyp')) and
            all(second.get(n) is None for n in SECTION2[4:]),
            'unsupported supplied assembly section options')
    thickness = second.values[:4]
    require(all(t is not None and t > 0 for t in thickness) and
            all(t == thickness[0] for t in thickness), 'assembly requires uniform positive thickness')
    # Reuse the source parser's finite SI conversion and exact binary operation.
    from .keyword_cards import _scale
    return SectionShell(first.get('secid'), first.get('elform'), 3,
                        tuple(_scale(t, units.length_to_m) for t in thickness),
                        (first, second), block)


def compile_assembly_declarations(index, part_ids, units, limits):
    require(0 < len(part_ids) <= limits.parts and len(set(part_ids)) == len(part_ids) and
            all(type(p) is int and 0 < p < 2**63 for p in part_ids),
            'invalid assembly part selection or part cap')
    result = []
    for pid in sorted(part_ids):
        part = parse_part(index.one('part', pid))
        section = assembly_section(index.one('section', part.section_id), units)
        material = parse_material(index.one('material', part.material_id), units)
        curve = parse_curve(index.one('curve', material.hardening_curve_id), units)
        result.append(PartDeclarations(part, section, material, curve, units))
    return tuple(result)


DONOR_POLICY = dict(
    revision='a62b27e6baa555d222a580d6218867d0be4d70b5',
    source='reader/source/dyna2rad/dyna2rad/_private/convertprops.cxx:776-820',
    source_sha256='87ce68f7e6d9d9186bb2ed5bbc74d1ca97226b8ebaffd48408e2becaae46852d',
    source_elforms=[2, 16], Ishell=24, Ish3n=2, NIP=3, Ismstr=2, ITHICK=1, IPLAS=1,
    interpretation='Explicit ordinary MAT024 converter policy; source ELFORM remains unchanged',
    ls_dyna_formulation_equivalence_qualified=False, assembly_mechanics_qualified=False)
