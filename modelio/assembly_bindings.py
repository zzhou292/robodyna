"""Explicit source-parent bindings for future resident collection composition."""
from ._legacy import require


def compile_parent_bindings(declarations, geometry):
    nodes = {nid: i for i, nid in enumerate(geometry.source_node_ids)}
    material = {mid: i for i, mid in enumerate(sorted({d.material.material_id for d in declarations}))}
    section = {sid: i for i, sid in enumerate(sorted({d.section.section_id for d in declarations}))}
    curve = {cid: i for i, cid in enumerate(sorted({d.hardening_curve.curve_id for d in declarations}))}
    by_part = {d.part.part_id: d for d in declarations}
    result = []
    next_family = {'QEPH': 0, 'T3': 0}
    for part_index, part in enumerate(geometry.parts):
        declaration = by_part[part.part_id]
        require(part.material_id == declaration.material.material_id and
                part.section_id == declaration.section.section_id,
                'assembly parent declaration binding mismatch')
        for part_parent_index, shell in enumerate(part.shells):
            family = 'QEPH' if shell.arity == 4 else 'T3'
            result.append(dict(parent_index=len(result), part_index=part_index,
                               part_parent_index=part_parent_index, family=family,
                               family_index=next_family[family], source_element_id=shell.source_id,
                               source_part_id=part.part_id, source_material_id=part.material_id,
                               source_section_id=part.section_id,
                               source_curve_id=declaration.hardening_curve.curve_id,
                               source_elform=declaration.section.source_elform,
                               material_index=material[part.material_id], section_index=section[part.section_id],
                               curve_index=curve[declaration.hardening_curve.curve_id],
                               node_indices=[nodes[nid] for nid in shell.raw_record[2:2 + shell.arity]]))
            next_family[family] += 1
    require(len(result) == geometry.shell_count and
            next_family == {'QEPH': geometry.qeph_parent_count, 'T3': geometry.t3_parent_count},
            'incomplete assembly parent binding coverage')
    return result
