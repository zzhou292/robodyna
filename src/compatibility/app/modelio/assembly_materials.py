"""Explicit V3 constitutive dispatch over the existing authenticated declarations."""
from dataclasses import asdict

from ._legacy import require
from .assembly_law44 import compile_material as compile_law44, material_table as law44_table
from .law1_declarations import ElasticMaterial, parse_elastic_material, LAYERED_LAW1_POLICY

MATERIAL_POLICIES = ('tabulated', 'law44_tabulated_or_linear', 'layered_law1_or_law44')
SECTION_SCHEMA = 'robo-dyna.source-assembly-inventory.v3'


def compile_material(index, material_id, units, allow_linear, allow_elastic):
    block = index.one('material', material_id)
    if allow_elastic and block.keyword == '*MAT_ELASTIC':
        return parse_elastic_material(block, units), None
    return compile_law44(index, material_id, units, allow_linear)


def material_table(declarations):
    result = {}
    for declaration in declarations:
        material = declaration.material
        elastic = isinstance(material, ElasticMaterial)
        row = asdict(material) if elastic else law44_table((declaration,))[0]
        row['material_law'] = 'layered_law1' if elastic else 'layered_law44'
        require(material.material_id not in result or result[material.material_id] == row,
                'conflicting assembly material identity')
        result[material.material_id] = row
    return [row for _, row in sorted(result.items())]


def section_policy():
    return dict(policy='layered_law1_or_law44', law1=dict(LAYERED_LAW1_POLICY))
