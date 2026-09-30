"""Explicit ordinary MAT024 FAIL-to-Johnson declaration; legacy parsers stay strict."""
from dataclasses import dataclass
from ._legacy import require
from .declarations import Mat024
from .keyword_cards import (_material_cards, _material_declaration,
                            _material_options, _positive, parse_curve)
from .law44_declarations import _parse_linear_law44_material

POLICY = dict(
    name='openradioss_mat024_positive_fail_constant_all_points_v1',
    revision='a62b27e6baa555d222a580d6218867d0be4d70b5',
    material_source='reader/source/dyna2rad/dyna2rad/_private/convertmats.cxx:6390-6424',
    failure_source='reader/source/dyna2rad/dyna2rad/_private/convertmats.cxx:1485-1494,1715-1724',
    point_eps_max_cleared=True, failure='ConstantAllPoints', D1='FAIL', D2=0, D3=0, D4=0, D5=0,
    IFAIL_SH=2, NIP=3, source_time_to_s=1, rate_filter_hz=10000,
    table_continuation='NativeLastSegment',
    tabulated_etan='blank_or_zero_preserved_native_B_zero', simulation_ready=False)


@dataclass(frozen=True)
class ConstantFailureMaterial:
    material: Mat024
    failure_strain: float
    hardening_model: str


def parse_constant_failure_material(block, units):
    require(units.time_to_s == 1, 'constant failure direct import requires source seconds')
    cards = _material_cards(block)
    first, second, eps, stress = cards
    _positive(first.get('fail'))
    require(second.get('vp') == 0, 'constant failure requires supplied total-rate VP=0')
    require(first.get('pr') is not None and 0 <= first.get('pr') < .5,
            'constant failure requires nonnegative Poisson ratio')
    if second.get('lcss') == 0:
        material = _parse_linear_law44_material(block, units, constant_failure=True).material
        kind = 'law44_linear'
    else:
        _positive(first.get('mid'), first.get('ro'), first.get('e'),
                  second.get('lcss'), second.get('c'), second.get('p'))
        _material_options(*cards, constant_failure=True)
        require(first.get('sigy') is not None and first.get('etan') in (None, 0),
                'tabulated failure material requires supplied SIGY and blank or zero ETAN')
        material = _material_declaration(block, units, cards)
        kind = 'law44_tabulated'
    return ConstantFailureMaterial(material, first.get('fail'), kind)


def compile_constant_failure_material(index, material_id, units):
    parsed = parse_constant_failure_material(index.one('material', material_id), units)
    curve_id = parsed.material.hardening_curve_id
    curve = parse_curve(index.one('curve', curve_id), units) if curve_id else None
    return parsed, curve
