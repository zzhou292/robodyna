"""Explicit V2 analytic/table material selection for the existing assembly compiler."""
from dataclasses import asdict

from .keyword_cards import parse_material, parse_curve
from .law44_declarations import parse_linear_law44_material


def compile_material(index, material_id, units, allow_linear):
    block = index.one('material', material_id)
    try:
        material = parse_material(block, units)
    except ValueError:
        if not allow_linear:
            raise
        return parse_linear_law44_material(block, units).material, None
    return material, parse_curve(index.one('curve', material.hardening_curve_id), units)


def material_table(declarations):
    materials = {}
    for declaration in declarations:
        value = declaration.material
        row = asdict(value)
        row['hardening_model'] = 'law44_tabulated' if value.hardening_curve_id else 'law44_linear'
        previous = materials.setdefault(value.material_id, row)
        if previous != row:
            raise ValueError('conflicting assembly material identity')
    return [value for _, value in sorted(materials.items())]
