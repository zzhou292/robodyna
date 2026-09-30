"""Explicit original MAT123/NUMINT1 and NLOC resolution; no owner admission."""
from dataclasses import asdict
from copy import deepcopy
import math

from ._legacy import require
from .declarations import SectionShell
from .keyword_cards import (MAT1, SECTION1, SECTION2, _blank, _positive,
                            _scale, _shape, card, parse_part)

KEYWORDS = ('*MAT_MODIFIED_PIECEWISE_LINEAR_PLASTICITY', '*MAT_123')
MAT2 = ('c', 'p', 'lcss', 'lcsr', 'vp', 'epsthin', 'epsmaj', 'numint')
SCHEMA = 'robo-dyna.vehicle-glass-declarations.v1'
POLICY = dict(
    name='original_mat123_numint1_native_tab1_placement_v1',
    revision='a62b27e6baa555d222a580d6218867d0be4d70b5',
    material_source='convertmats.cxx:6170-6220,6390-6424',
    failure='Tab1AnyPoint', source_NUMINT=1, converter_IFAIL_SH=2,
    resolved_IFAIL_SH=1, resolved_PTHKF=1e-6, NIP=3,
    rate_policy='FilteredZeroC', resolved_C=0, resolved_P=1,
    resolved_VP=2, rate_filter_hz=10000, source_time_to_s=1,
    point_eps_max_cleared=True, FLD=False,
    placement_policy='original_nloc_native_type1_reference_plane_v1',
    nloc_to_ipos=[[-1, 4], [0, 0], [1, 3]], contact_projection=False,
    simulation_ready=False)


def glass_material(block, units):
    _shape(block, KEYWORDS, 4)
    require(units.time_to_s == 1, 'glass filter resolution requires source seconds')
    first = card(block.cards[0], MAT1, [int] + [float] * 7)
    second = card(block.cards[1], MAT2)
    eps = card(block.cards[2], tuple(f'eps{i}' for i in range(1, 9)))
    stress = card(block.cards[3], tuple(f'es{i}' for i in range(1, 9)))
    _positive(first.get('mid'), first.get('ro'), first.get('e'),
              first.get('sigy'), first.get('fail'))
    require(first.get('pr') is not None and 0 <= first.get('pr') < .5,
            'glass requires nonnegative Poisson ratio below .5')
    tangent = first.get('etan')
    require(tangent is not None and 0 <= tangent < first.get('e'), 'invalid glass ETAN')
    _blank(first, ('tdel',))
    _blank(second, ('c', 'p', 'vp', 'epsthin', 'epsmaj'))
    require(second.get('lcss') == 0 and second.get('lcsr') == 0 and second.get('numint') == 1,
            'glass requires explicit LCSS0/LCSR0/NUMINT1')
    _blank(eps, eps.names)
    _blank(stress, stress.names)
    young = _scale(first.get('e'), units.stress_to_si)
    tangent = _scale(tangent, units.stress_to_si)
    hardening = tangent * young / (young - tangent)
    require(math.isfinite(hardening) and hardening >= 0 and (tangent == 0 or hardening > 0),
            'glass analytic modulus overflow or underflow')
    return dict(material_id=first.get('mid'), material_law='layered_law44',
                hardening_model='law44_linear',
                density_kg_m3=_scale(first.get('ro'), units.density_to_si), young_pa=young,
                poisson_ratio=first.get('pr'), supplied_sigy_pa=_scale(first.get('sigy'), units.stress_to_si),
                supplied_etan_pa=tangent, failure_strain=first.get('fail'),
                source_numint=second.get('numint'),
                cards=[asdict(c) for c in (first, second, eps, stress)], source=asdict(block))


def glass_section(block, units):
    _shape(block, ('*SECTION_SHELL',), 2)
    first = card(block.cards[0], SECTION1, [int, int, float, int, float, float, int, int])
    second = card(block.cards[1], SECTION2, [float] * 6 + [int, int])
    _positive(first.get('secid'))
    require(first.get('elform') == 2 and first.get('nip') == 3, 'glass requires ELFORM2/NIP3')
    _blank(first, ('shrf', 'propt', 'qr_irid', 'icomp', 'setyp'))
    _blank(second, ('marea', 'idof', 'edgset'))
    nloc = second.get('nloc')
    require(nloc in (None, -1, 0, 1), 'unsupported original glass NLOC')
    thickness = second.values[:4]
    _positive(*thickness)
    require(all(t == thickness[0] for t in thickness), 'glass requires uniform thickness')
    section = SectionShell(first.get('secid'), 2, 3,
                           tuple(_scale(t, units.length_to_m) for t in thickness),
                           (first, second), block)
    placement = 'bottom_reference_plane' if nloc == -1 else (
        'top_reference_plane' if nloc == 1 else 'centered')
    return dict(asdict(section), source_nloc=nloc, placement=placement)


def compile_glass_part(index, part_id, units):
    part = parse_part(index.one('part', part_id))
    return dict(part=asdict(part),
                material=glass_material(index.one('material', part.material_id), units),
                section=glass_section(index.one('section', part.section_id), units))


def extend_glass_resolution(index, units, result):
    """Extend the separate resolution, retaining its V1 declarations and all rows."""
    declarations = []
    counts = result['counts']
    counts.update(glass_parts=0, glass_shells=0, placed_glass_shells=0)
    for row in result['parts']:
        if row['status'] != 'unresolved':
            continue
        try:
            declaration = compile_glass_part(index, row['part_id'], units)
        except (ValueError, OverflowError):
            continue
        row['status'] = 'glass_tab1'
        declarations.append(declaration)
        counts['glass_parts'] += 1
        counts['glass_shells'] += row['shells']
        counts['unresolved_parts'] -= 1
        counts['unresolved_shells'] -= row['shells']
        if declaration['section']['placement'] != 'centered':
            counts['placed_glass_shells'] += row['shells']
    require(declarations, 'glass resolution contains no supported original MAT123 parts')
    result['schema'] = 'robo-dyna.vehicle-section-resolution.v2'
    result['glass_declarations'] = dict(schema=SCHEMA, policy=deepcopy(POLICY), parts=declarations)
    return result
