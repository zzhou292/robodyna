"""Compact declaration-only plan; canonical geometry and source mechanics stay separate."""
from dataclasses import asdict
from ._legacy import require
from .assembly_declarations import assembly_section
from .assembly_materials import compile_material, material_table, section_policy, SECTION_SCHEMA
from .declarations import PartDeclarations
from .keyword_cards import parse_part
from .law44_declarations import LINEAR_LAW44_POLICY

SCHEMA = 'robo-dyna.vehicle-source-declarations.v1'
MAX_BYTES = 8 * 1024 * 1024


def compile_vehicle_declarations(index, scope, units, authority):
    require(scope['schema'] == 'robo-dyna.full-shell-scope.v1' and
            index.sha256 == scope['source']['member_sha256'], 'vehicle declaration source mismatch')
    selected = scope['selected_shell_part_ids']
    require(selected == sorted(set(selected)) and 0 < len(selected) <= 1024,
            'invalid complete vehicle part selection')
    source_parts = {p['source_part_id']: p for p in scope['declarations']['parts']}
    require(len(source_parts) == len(scope['declarations']['parts']), 'duplicate source part')
    tables = {kind: {r['identity']: r for r in rows}
              for kind, rows in scope['declarations']['tables'].items()}
    rows, supported, counts = [], [], dict(parts=0, shells=0, supported_parts=0, supported_shells=0)
    for pid in selected:
        original = source_parts[pid]
        require(original['retained_shell_part'], 'selected part lacks source coverage')
        sid, mid = original['source_section_id'], original['source_material_id']
        blocks = {kind: index.one(kind, identity) for kind, identity in
                  (('part', pid), ('section', sid), ('material', mid))}
        require(blocks['part'].sha256 == original['source_part_sha256'] and
                all(blocks[k].sha256 == tables[k][i]['sha256']
                    for k, i in (('section', sid), ('material', mid))), 'source declaration block changed')
        failures, parsed = [], {}
        operations = (('part', lambda: parse_part(blocks['part'])),
                      ('section', lambda: assembly_section(blocks['section'], units)),
                      ('material', lambda: compile_material(index, mid, units, True, True)))
        for stage, operation in operations:
            try:
                parsed[stage] = operation()
            except (ValueError, OverflowError) as error:
                failures.append(dict(stage=stage, reason=str(error)))
        row = dict(part_id=pid, material_id=mid, section_id=sid, shells=original['counts']['shells'],
                   status='unresolved' if failures else 'supported_declaration', obligations=failures)
        if failures:
            row['source_blocks'] = {k: asdict(v) for k, v in blocks.items()}
        else:
            material, curve = parsed['material']
            require(parsed['part'].part_id == pid and parsed['part'].material_id == mid and
                    parsed['part'].section_id == sid, 'parsed part association changed')
            supported.append(PartDeclarations(parsed['part'], parsed['section'], material, curve, units))
            counts['supported_parts'] += 1
            counts['supported_shells'] += row['shells']
        rows.append(row); counts['parts'] += 1; counts['shells'] += row['shells']
    def table(attribute, identity):
        values = {}
        for declaration in supported:
            value = getattr(declaration, attribute)
            if value is None: continue
            key = getattr(value, identity)
            require(key not in values or values[key] == value, 'conflicting typed declaration')
            values[key] = value
        return [asdict(value) for _, value in sorted(values.items())]
    require(counts['shells'] == scope['coverage']['retained']['shells'], 'complete shell coverage changed')
    require(supported, 'vehicle declaration plan has no supported source declarations')
    declarations = dict(parts=table('part', 'part_id'), sections=table('section', 'section_id'),
                        materials=material_table(supported), curves=table('hardening_curve', 'curve_id'), units=asdict(units))
    return dict(schema=SCHEMA, simulation_ready=False, native_startup_qualified=False,
                source=authority, counts=counts, parts=rows,
                supported_declarations=dict(schema=SECTION_SCHEMA, selected_part_ids=[d.part.part_id for d in supported],
                    declarations=declarations, section_policy=section_policy(), law44_policy=dict(LINEAR_LAW44_POLICY)))
