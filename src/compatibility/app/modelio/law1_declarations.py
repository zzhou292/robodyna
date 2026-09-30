"""Source elastic coefficients for the qualified layered LAW1 section.

This reader retains only supplied elastic fields. Source geometry, section,
attachments and actual resident admission remain separate obligations.
"""
from dataclasses import dataclass

from ._legacy import require
from .keyword_cards import card, _shape, _blank, _positive, _scale
from .source_blocks import SourceBlock


ELASTIC_FIELDS = ('mid', 'ro', 'e', 'pr', 'reserved5', 'reserved6', 'reserved7', 'reserved8')


@dataclass(frozen=True)
class ElasticMaterial:
    material_id: int
    density_kg_m3: float
    young_pa: float
    poisson_ratio: float
    cards: tuple
    source: SourceBlock


def parse_elastic_material(block, units):
    """Strict original MAT_ELASTIC card, with no invented plastic/rate fields."""
    _shape(block, ('*MAT_ELASTIC',), 1)
    fields = card(block.cards[0], ELASTIC_FIELDS, [int] + [float] * 7)
    _positive(fields.get('mid'), fields.get('ro'), fields.get('e'))
    poisson = fields.get('pr')
    require(poisson is not None and 0 <= poisson < .5,
            'Qualified layered LAW1 requires explicit 0 <= Poisson ratio < 0.5')
    # Optional columns are outside this source subset, including explicit zero.
    _blank(fields, ELASTIC_FIELDS[4:])
    return ElasticMaterial(fields.get('mid'), _scale(fields.get('ro'), units.density_to_si),
                           _scale(fields.get('e'), units.stress_to_si), poisson, (fields,), block)


def layered_law1_candidates(index, parts, units):
    result = []
    for part in parts:
        if not part['retained_shell_part']:
            continue
        block = index.one('material', part['source_material_id'])
        try:
            material = parse_elastic_material(block, units)
        except (ValueError, OverflowError):
            continue
        result.append(dict(source_part_id=part['source_part_id'], source_material_id=material.material_id,
                           source_material_sha256=block.sha256, shells=part['counts']['shells'],
                           young_pa=material.young_pa, poisson_ratio=material.poisson_ratio,
                           density_kg_m3=material.density_kg_m3,
                           part_and_section_candidate=not any(r['family'] in ('part', 'section')
                               for r in part['existing_adapter_rejections'])))
    return dict(policy=LAYERED_LAW1_POLICY, parts=result,
                shells=sum(p['shells'] for p in result),
                shells_with_supported_part_and_section=sum(p['shells'] for p in result
                    if p['part_and_section_candidate']), simulation_ready=False)


LAYERED_LAW1_POLICY = dict(
    revision='a62b27e6baa555d222a580d6218867d0be4d70b5',
    material_source='reader/source/dyna2rad/dyna2rad/_private/convertmats.cxx:218-239',
    section_source='reader/source/dyna2rad/dyna2rad/_private/convertprops.cxx:776-813',
    material_law='layered_law1', NIP=3, ITHICK=1,
    interpretation='Supplied elastic coefficients; native layered LAW1 section, no plastic history',
    case_integration_qualified=False)
