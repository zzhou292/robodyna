"""Add narrowly qualified declarations without changing historical VehicleSourcePlan V1."""
from dataclasses import asdict
import hashlib
import json
from ._legacy import require
from .assembly_declarations import assembly_section
from .assembly_materials import section_policy, SECTION_SCHEMA
from .constant_failure_declarations import compile_constant_failure_material, POLICY
from .keyword_cards import parse_part
from .law44_declarations import LINEAR_LAW44_POLICY
from .vehicle_declarations import compile_vehicle_declarations

SCHEMA = 'robo-dyna.vehicle-section-resolution.v1'
MAX_BYTES = 4 * 1024 * 1024


def _encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode()


def compile_vehicle_section_resolution(index, scope, units, authority):
    legacy = compile_vehicle_declarations(index, scope, units, authority)
    original_bytes = _encoded(legacy)
    rows, parts, sections, materials, curves = [], {}, {}, {}, {}
    counts = dict(parts=len(legacy['parts']), shells=legacy['counts']['shells'],
                  existing_parts=legacy['counts']['supported_parts'],
                  existing_shells=legacy['counts']['supported_shells'],
                  failure_parts=0, failure_shells=0, unresolved_parts=0, unresolved_shells=0)
    for old in legacy['parts']:
        pid = old['part_id']
        row = {key: old[key] for key in ('part_id', 'material_id', 'section_id', 'shells')}
        if old['status'] == 'supported_declaration':
            row['status'] = 'existing'
        else:
            try:
                part = parse_part(index.one('part', pid))
                section = assembly_section(index.one('section', part.section_id), units)
                parsed, curve = compile_constant_failure_material(index, part.material_id, units)
            except (ValueError, OverflowError):
                row['status'] = 'unresolved'
                counts['unresolved_parts'] += 1
                counts['unresolved_shells'] += row['shells']
            else:
                row['status'] = 'constant_failure'
                counts['failure_parts'] += 1
                counts['failure_shells'] += row['shells']
                parts[pid] = asdict(part)
                sections[section.section_id] = asdict(section)
                material = asdict(parsed.material)
                material.update(material_law='layered_law44', hardening_model=parsed.hardening_model,
                                failure_strain=parsed.failure_strain)
                previous = materials.setdefault(part.material_id, material)
                require(previous == material, 'conflicting resolved failure material')
                if curve:
                    value = asdict(curve)
                    require(curves.setdefault(curve.curve_id, value) == value,
                            'conflicting resolved hardening curve')
        rows.append(row)
    require(parts, 'vehicle resolution contains no ordinary constant-failure declarations')
    resolved = dict(schema=SECTION_SCHEMA, selected_part_ids=sorted(parts),
                    section_policy=section_policy(), law44_policy=dict(LINEAR_LAW44_POLICY),
                    declarations=dict(units=asdict(units),
                        parts=[parts[k] for k in sorted(parts)], sections=[sections[k] for k in sorted(sections)],
                        materials=[materials[k] for k in sorted(materials)], curves=[curves[k] for k in sorted(curves)]))
    source = dict(authority, vehicle_plan_bytes=len(original_bytes),
                  vehicle_plan_sha256=hashlib.sha256(original_bytes).hexdigest())
    return dict(schema=SCHEMA, source=source, policy=dict(POLICY), counts=counts, parts=rows,
                constant_failure_declarations=resolved, simulation_ready=False,
                native_startup_qualified=False)
