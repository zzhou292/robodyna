"""Strict native analytic MAT024 declaration; no curve synthesis or case admission."""
from dataclasses import dataclass
import math

from ._legacy import require
from .declarations import Mat024
from .keyword_cards import (_material_cards, _material_declaration,
                            _material_options, _positive)


@dataclass(frozen=True)
class LinearLaw44Material:
    material: Mat024
    plastic_hardening_pa: float
    hardening_model: str = 'law44_linear'


def parse_linear_law44_material(block, units):
    """Preserve source scalars/cards and resolve B=ETAN*E/(E-ETAN) in SI.

    This first branch requires explicit total-rate VP0 and positive C/P. Failure,
    blank-rate defaults, inline points and section formulation are separate gates.
    The legacy positive-curve parser remains unchanged in its admission domain.
    """
    return _parse_linear_law44_material(block, units)


def _parse_linear_law44_material(block, units, *, constant_failure=False):
    require(units.time_to_s == 1, 'analytic LAW44 default filter currently requires source seconds')
    cards = _material_cards(block)
    first, second, eps, stress = cards
    _positive(first.get('mid'), first.get('ro'), first.get('e'), first.get('sigy'),
              second.get('c'), second.get('p'))
    _material_options(*cards, constant_failure=constant_failure)
    require(second.get('lcss') == 0, 'analytic LAW44 requires explicit LCSS=0')
    require(second.get('vp') == 0, 'analytic LAW44 requires explicit total-rate VP=0')
    require(first.get('pr') >= 0, 'analytic LAW44 requires nonnegative Poisson ratio')
    tangent = first.get('etan')
    require(tangent is not None and 0 <= tangent < first.get('e'),
            'analytic LAW44 requires supplied 0 <= ETAN < E')
    material = _material_declaration(block, units, cards)
    young, tangent = material.young_pa, material.supplied_etan_pa
    require(tangent < young, 'analytic LAW44 SI tangent must remain below E')
    hardening = tangent * young / (young - tangent)
    require(math.isfinite(hardening) and hardening >= 0 and (tangent == 0 or hardening > 0),
            'analytic LAW44 plastic modulus overflow or underflow')
    return LinearLaw44Material(material, hardening)


def linear_law44_candidates(index, parts, units):
    """Report source candidates without changing existing adapter admission."""
    result = []
    for part in parts:
        if not part['retained_shell_part']:
            continue
        block = index.one('material', part['source_material_id'])
        try:
            parsed = parse_linear_law44_material(block, units)
        except (ValueError, OverflowError):
            continue  # The complete coverage table retains the original rejection.
        material = parsed.material
        result.append(dict(source_part_id=part['source_part_id'], source_material_id=material.material_id,
                           source_material_sha256=block.sha256, shells=part['counts']['shells'],
                           initial_yield_pa=material.supplied_sigy_pa,
                           tangent_modulus_pa=material.supplied_etan_pa,
                           plastic_hardening_pa=parsed.plastic_hardening_pa,
                           part_and_section_candidate=not any(r['family'] in ('part', 'section')
                               for r in part['existing_adapter_rejections'])))
    return dict(policy=LINEAR_LAW44_POLICY, parts=result,
                shells=sum(p['shells'] for p in result),
                shells_with_supported_part_and_section=sum(p['shells'] for p in result
                    if p['part_and_section_candidate']),
                simulation_ready=False)


LINEAR_LAW44_POLICY = dict(
    revision='a62b27e6baa555d222a580d6218867d0be4d70b5',
    source='reader/source/dyna2rad/dyna2rad/_private/convertmats.cxx:6390-6424',
    hardening_model='law44_linear', A='SIGY', B='ETAN*E/(E-ETAN)', n=1,
    function_reference=0, vp=0, source_time_to_s=1, rate_filter_hz=10000,
    interpretation='Native analytic declaration only; no synthesized hardening curve',
    case_integration_qualified=False)
