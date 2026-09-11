"""Original MAT100/CST1 declarations; native conversion remains in C++."""
from dataclasses import dataclass

from ._legacy import require
from .keyword_cards import card, _shape, _blank, _positive, parse_part

SECTION = ('secid', 'elform', 'shrf', 'qr_irid', 'cst', 'scoor', 'nsm', 'reserved8')
DIMENSIONS = ('ts1', 'ts2', 'tt1', 'tt2', 'nprint', 'reserved6', 'reserved7', 'reserved8')
MATERIAL = ('mid', 'ro', 'e', 'pr', 'sigy', 'et', 'dt', 'tfail')
FAILURE = ('efail', 'nrr', 'nrs', 'nrt', 'mrr', 'mss', 'mtt', 'nf')


@dataclass(frozen=True)
class BeamPropertyDeclaration:
    part: object
    section_source: object
    material_source: object
    section_cards: tuple
    material_cards: tuple


def parse_beam_property(part_block, section_block, material_block):
    part = parse_part(part_block)
    _shape(section_block, ('*SECTION_BEAM',), 2)
    _shape(material_block, ('*MAT_SPOTWELD', '*MAT_100'), 2)
    section = card(section_block.cards[0], SECTION,
                   [int, int, float, int, int, int, float, float])
    dimensions = card(section_block.cards[1], DIMENSIONS)
    material = card(material_block.cards[0], MATERIAL, [int] + [float] * 7)
    failure = card(material_block.cards[1], FAILURE)
    require(section.get('secid') == part.section_id and material.get('mid') == part.material_id,
            'TYPE13 property source IDs disagree with PART')
    require(section.get('elform') == 9 and section.get('cst') == 1 and
            section.get('shrf') == 1 and section.get('qr_irid') == 0,
            'TYPE13 declaration requires explicit ELFORM9/CST1/SHRF1/QR0')
    _blank(section, SECTION[5:])
    _positive(dimensions.get('ts1'), dimensions.get('ts2'))
    _blank(dimensions, DIMENSIONS[2:])
    _positive(material.get('ro'), material.get('e'), material.get('sigy'), material.get('et'),
              material.get('tfail'), failure.get('efail'))
    require(material.get('pr') is not None and 0 <= material.get('pr') < .5 and material.get('et') < material.get('e'),
            'Unsupported source Poisson ratio or tangent modulus')
    _blank(material, ('dt',))
    _blank(failure, FAILURE[1:])
    return BeamPropertyDeclaration(part, section_block, material_block,
                                   (section, dimensions), (material, failure))


TYPE13_POLICY = dict(
    revision='a62b27e6baa555d222a580d6218867d0be4d70b5',
    converter_sha256='87ce68f7e6d9d9186bb2ed5bbc74d1ca97226b8ebaffd48408e2becaae46852d',
    converter='ConvertSectionBeamToSpringBeam', section='ELFORM9/CST1',
    unit_authority='original README t/mm/s; missing CONTROL_UNITS is not a converter unit declaration',
    resolved=dict(TT1=0, TT2=0, Ileng=1, H=1, Ifail=1, Ifail2=0,
                  A=1, LSCALE=1, damping=0, alpha=1, beta=2, sensor=0, rate_failure=0),
    ignored_supplied_TFAIL='native converter sensor construction is commented out',
    native_conversion_owner='robo-dyna C++ modelio/type13/ConvertProperty.cpp',
    recurrence_qualified=False, endpoint_ties_qualified=False, simulation_ready=False)
