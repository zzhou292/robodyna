"""Strict declaration domain of the selected source connector; no solver map.

Nullable fields preserve absence. The legacy parser's zero placeholders are
removed using its blank mask before any interpretation. Optional source flags
are never materialized as solver defaults here.
"""
from decimal import Decimal, InvalidOperation
import math

from ._legacy import fields, require, WallImportError
from .declarations import TypedCard, Part, SectionShell, Mat024, HardeningCurve, PartDeclarations

PART = ('pid', 'secid', 'mid', 'eosid', 'hgid', 'grav', 'adpopt', 'tmid')
SECTION1 = ('secid', 'elform', 'shrf', 'nip', 'propt', 'qr_irid', 'icomp', 'setyp')
SECTION2 = ('t1', 't2', 't3', 't4', 'nloc', 'marea', 'idof', 'edgset')
MAT1 = ('mid', 'ro', 'e', 'pr', 'sigy', 'etan', 'fail', 'tdel')
MAT2 = ('c', 'p', 'lcss', 'lcsr', 'vp', 'reserved6', 'reserved7', 'reserved8')
CURVE = ('lcid', 'sidr', 'sfa', 'sfo', 'offa', 'offo', 'dattyp', 'lcint')


def integral_flag(text):
    """Accept exactly integral decimal flags, including source VP='0.0'."""
    try:
        number = Decimal(text)
        if (not number.is_finite() or number < -(2**31) or number >= 2**31 or
                number != number.to_integral_value()):
            raise ValueError('flag is nonfinite, fractional or out of range')
        return int(number)
    except InvalidOperation as error:
        raise ValueError('invalid decimal flag') from error


def card(source, names, converters=None, width=10):
    converters = converters or [float] * len(names)
    values, mask = fields(source.text, [width] * len(names), converters, [0] * len(names))
    return TypedCard(source.source_line, source.text, tuple(names),
                     tuple(None if mask & (1 << i) else value for i, value in enumerate(values)), mask)


def _shape(block, keywords, count):
    require(block.keyword in keywords, f'unsupported keyword option {block.keyword}')
    require(len(block.cards) == count, f'{block.keyword}: expected exactly {count} data cards, including blanks')


def _blank(card_value, names):
    require(all(card_value.get(name) is None for name in names),
            f'line {card_value.source_line}: unsupported supplied fields {names}')


def _positive(*values):
    require(all(v is not None and math.isfinite(v) and v > 0 for v in values),
            'required value must be supplied, finite and positive')


def _scale(value, factor):
    result = value * factor
    require(math.isfinite(result), 'SI conversion overflow')
    require(value == 0 or result != 0, 'SI conversion underflow')
    return result


def parse_part(block):
    _shape(block, ('*PART',), 2)
    identity = card(block.cards[1], PART, [int] * 8)
    pid, sid, mid = identity.values[:3]
    _positive(pid, sid, mid)
    _blank(identity, PART[3:])
    title = block.cards[0].text
    require(bool(title.strip()) and len(title) <= 80, 'invalid PART title')
    return Part(pid, sid, mid, title, (identity,), block)


def parse_section(block, units):
    _shape(block, ('*SECTION_SHELL',), 2)
    first = card(block.cards[0], SECTION1, [int, int, float, int, float, float, int, int])
    second = card(block.cards[1], SECTION2, [float] * 6 + [int, int])
    _positive(first.get('secid'))
    require(first.get('elform') == 2 and first.get('nip') == 3,
            'declaration domain requires explicit source ELFORM=2 and NIP=3')
    _blank(first, ('shrf', 'propt', 'qr_irid', 'icomp', 'setyp'))
    _blank(second, SECTION2[4:])
    thickness = second.values[:4]
    _positive(*thickness)
    require(all(t == thickness[0] for t in thickness), 'nonuniform source thickness is outside declaration domain')
    return SectionShell(first.get('secid'), 2, 3,
                        tuple(_scale(t, units.length_to_m) for t in thickness), (first, second), block)


def _material_cards(block):
    _shape(block, ('*MAT_024', '*MAT_PIECEWISE_LINEAR_PLASTICITY'), 4)
    first = card(block.cards[0], MAT1, [int] + [float] * 7)
    second = card(block.cards[1], MAT2, [float, float, int, int, integral_flag, float, float, float])
    eps = card(block.cards[2], tuple(f'eps{i}' for i in range(1, 9)))
    stress = card(block.cards[3], tuple(f'es{i}' for i in range(1, 9)))
    return first, second, eps, stress


def parse_material(block, units):
    first, second, eps, stress = _material_cards(block)
    _positive(first.get('mid'), first.get('ro'), first.get('e'), second.get('lcss'),
              second.get('c'), second.get('p'))
    _material_options(first, second, eps, stress)
    return _material_declaration(block, units, (first, second, eps, stress))


def _material_options(first, second, eps, stress):
    require(first.get('pr') is not None and -1 < first.get('pr') < .5, 'Poisson ratio must be explicit and admissible')
    require(second.get('vp') in (None, 0), 'only supplied total-rate VP=0 or unresolved blank VP is declared')
    _blank(first, ('fail', 'tdel'))
    _blank(second, ('lcsr', 'reserved6', 'reserved7', 'reserved8'))
    _blank(eps, eps.names)
    _blank(stress, stress.names)
    for name in ('sigy', 'etan'):
        value = first.get(name)
        require(value is None or value >= 0, f'unsupported negative supplied {name}')


def _material_declaration(block, units, cards):
    first, second, eps, stress = cards
    def stress_si(name):
        value = first.get(name)
        return None if value is None else _scale(value, units.stress_to_si)
    return Mat024(first.get('mid'), _scale(first.get('ro'), units.density_to_si),
                  stress_si('e'), first.get('pr'), stress_si('sigy'), stress_si('etan'),
                  second.get('lcss'), _scale(second.get('c'), 1 / units.time_to_s), second.get('p'), second.get('vp'),
                  (first, second, eps, stress), block)


def parse_curve(block, units):
    require(block.keyword == '*DEFINE_CURVE', f'unsupported keyword option {block.keyword}')
    require(3 <= len(block.cards) <= 1025, 'hardening curve requires 2..1024 points')
    header = card(block.cards[0], CURVE, [int, int, float, float, float, float, int, int])
    _positive(header.get('lcid'))
    require(header.get('sidr') == 0, 'only explicit curve SIDR=0 is declared')
    _blank(header, ('dattyp', 'lcint'))
    defaults = []
    for name, default in (('sfa', 1.0), ('sfo', 1.0), ('offa', 0.0), ('offo', 0.0)):
        value = header.get(name)
        require(value is None or value == default, 'nonidentity curve transforms are outside declaration domain')
        if value is None:
            defaults.append((name, default))
    points = tuple(card(row, ('plastic_strain', 'stress'), width=20) for row in block.cards[1:])
    x = tuple(row.values[0] for row in points)
    y = tuple(row.values[1] for row in points)
    require(all(a is not None and a >= 0 for a in x) and x[0] == 0 and
            all(b > a for a, b in zip(x, x[1:])), 'hardening strain must start at zero and strictly increase')
    _positive(*y)
    require(all(b >= a for a, b in zip(y, y[1:])), 'softening curve is outside declaration domain')
    return HardeningCurve(header.get('lcid'), x, tuple(_scale(v, units.stress_to_si) for v in y),
                          tuple(defaults), (header,) + points, block)


def compile_part_declarations(index, part_id, units):
    def located(parser, block, *args):
        try:
            return parser(block, *args)
        except ValueError as error:
            raise WallImportError(f'{block.filename}:{block.first_line} {block.keyword}: {error}') from error
    part = located(parse_part, index.one('part', part_id))
    section = located(parse_section, index.one('section', part.section_id), units)
    material = located(parse_material, index.one('material', part.material_id), units)
    curve = located(parse_curve, index.one('curve', material.hardening_curve_id), units)
    return PartDeclarations(part, section, material, curve, units)
