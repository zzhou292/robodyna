"""Bounded original Yaris whole-part inventory and explicit extraction boundary.

All source validation finishes before create-only publication. No model is
admitted to dynamics; this package does not change TL capacities or state.
"""
from dataclasses import asdict
import json
from pathlib import Path
import zipfile

from ._legacy import require, file_sha256
from . import yaris_part
from .assembly_declarations import AssemblyLimits, compile_assembly_declarations, DONOR_POLICY
from .assembly_geometry import load_assembly_geometry
from .assembly_attachments import compile_assembly_attachments
from .assembly_bindings import compile_parent_bindings
from .canonical_geometry import read_json
from .source_blocks import (scan_metadata, DECLARATION_FAMILIES, ATTACHMENT_FAMILIES,
                            MAX_SOURCE_BYTES)
from .spotweld_cards import scan_spotwelds

YARIS_CONNECTOR_PARTS = (2000119, 2000120, 2000145, 2000157, 2000165, 2000260)
AUTHENTICATION_PART_ID = 2000157
MAX_REPORT_BYTES = 16 * 1024 * 1024


def _table(declarations, attribute, identity):
    result = {}
    for declaration in declarations:
        value = getattr(declaration, attribute)
        if value is None:
            continue
        key = getattr(value, identity)
        require(key not in result or result[key] == value, 'conflicting assembly declaration identity')
        result[key] = value
    return [asdict(value) for _, value in sorted(result.items())]


def compile_archive_assembly(archive_path, asset_dir, part_ids=YARIS_CONNECTOR_PARTS,
                             limits=AssemblyLimits(), boundary_policy='unassigned',
                             material_policy='tabulated'):
    require(isinstance(limits, AssemblyLimits), 'explicit offline AssemblyLimits required')
    require(material_policy in ('tabulated', 'law44_tabulated_or_linear'), 'unsupported assembly material policy')
    allow_linear = material_policy == 'law44_tabulated_or_linear'
    part_ids = tuple(part_ids)
    require(boundary_policy in ('unassigned', 'released_external_connections'),
            'unsupported assembly extraction boundary policy')
    require(0 < len(part_ids) <= limits.parts and
            all(type(p) is int and 0 < p < 2**63 for p in part_ids) and len(set(part_ids)) == len(part_ids),
            'invalid assembly part IDs or part cap')
    # Reuse the established archive/README/unit authentication without altering
    # its API or generator hashes. The pinned connector is solely an archive
    # authentication/units anchor. It is not automatically selected or loaded;
    # ELFORM16-only selections still use their own complete declarations below.
    seed, authority = yaris_part._compile_archive_part(archive_path, AUTHENTICATION_PART_ID)
    reference = read_json(yaris_part.REFERENCE)
    member = reference['archive_member_prefix'] + yaris_part.VEHICLE
    with zipfile.ZipFile(archive_path) as archive:
        yaris_part._member(archive, member, MAX_SOURCE_BYTES)
        with archive.open(member) as stream:
            index = scan_metadata(stream, yaris_part.VEHICLE,
                                  families=DECLARATION_FAMILIES | ATTACHMENT_FAMILIES)
        with archive.open(member) as stream:
            weld_inventory = scan_spotwelds(stream, yaris_part.VEHICLE)
    require(index.sha256 == weld_inventory.source_sha256 == reference['files'][yaris_part.VEHICLE]['sha256'],
            'assembly source member SHA256 mismatch')
    declarations = compile_assembly_declarations(index, part_ids, seed.units, limits, allow_linear)
    geometry = load_assembly_geometry(asset_dir, archive_path, declarations, reference, limits)
    attachments = compile_assembly_attachments(index, weld_inventory, geometry, asset_dir,
                                                archive_path, reference, limits)
    groups = attachments['nodal_rigid_groups']
    welds = attachments['spotwelds']
    external_ties = [dict(source_identity=t['contact']['source_identity'],
                         selected_slave_part_ids=t['selected_slave_part_ids'],
                         selected_master_part_ids=t['selected_master_part_ids'], actual_pairing_qualified=False,
                         reason='only one side contains selected parts; every possible selected tie crosses the extraction boundary')
                     for t in attachments['tied_candidates'] if t['candidate_pair_scope'] == 'cross_boundary_only']
    boundary = dict(policy=boundary_policy, applied_to_dynamics=False, mass_removed_from_selected_parts=False,
                    internal_connections='retain every selected internal nodal-rigid group and spotweld',
                    outgoing_nodal_rigid_ids=[g['rigid']['identity'] for g in groups if g['classification'] == 'outgoing'],
                    outgoing_spotweld_ids=[w['weld']['identity'] for w in welds if w['classification'] == 'outgoing'],
                    outgoing_tied_source_scopes=external_ties,
                    released_external_tied_source_scopes=external_ties
                    if boundary_policy == 'released_external_connections' else [],
                    unresolved_tied_scope='retained; no edge/node pairs or forces have been invented',
                    interpretation=('future extracted component with deliberately released external interfaces; '
                                    'not a source vehicle assembly or a full closure claim')
                    if boundary_policy == 'released_external_connections' else 'no dynamics boundary has been assigned')
    names = ('assembly_declarations.py', 'assembly_geometry.py', 'assembly_attachments.py', 'assembly_bindings.py',
             'spotweld_cards.py', 'yaris_assembly.py', 'canonical_geometry.py', 'canonical_incidence.py',
             'source_blocks.py', 'attachment_cards.py', 'keyword_cards.py', 'declarations.py', '_legacy.py', 'yaris_part.py',
             'assembly_law44.py', 'law44_declarations.py')
    generator = {'modelio/' + name: file_sha256(Path(__file__).with_name(name)) for name in names}
    for name in ('compile_yaris_assembly.py', 'compile_yaris_part.py', 'import_yaris_vehicle.py', 'import_yaris_wall.py'):
        generator['tools/' + name] = file_sha256(yaris_part.ROOT / 'tools' / name)
    result = dict(schema='robo-dyna.source-assembly-inventory.v2' if allow_linear else
                  'robo-dyna.source-assembly-inventory.v1', simulation_ready=False,
                full_attachment_closure_qualified=False, geometry_modified=False, mechanics_capacity_changed=False,
                source_mass_equivalence_qualified=False,
                source=dict(authority['source'], all_other_entities='outside the declared whole-part and literal interface scope',
                            authentication_anchor_part_id=AUTHENTICATION_PART_ID,
                            authentication_anchor_automatically_selected=False),
                offline_limits=asdict(limits), selected_part_ids=sorted(part_ids),
                counts=dict(parts=len(declarations), shells=geometry.shell_count, nodes=len(geometry.source_node_ids),
                            q4=geometry.qeph_parent_count, native_t3=geometry.t3_parent_count,
                            shared_nodes=len(geometry.shared_node_ids), materials=len({d.material.material_id for d in declarations}),
                            sections=len({d.section.section_id for d in declarations}),
                            curves=len({d.hardening_curve.curve_id for d in declarations if d.hardening_curve is not None}),
                            internal_nodal_rigid_groups=sum(g['classification'] == 'internal' for g in groups),
                            outgoing_nodal_rigid_groups=sum(g['classification'] == 'outgoing' for g in groups),
                            internal_spotwelds=sum(w['classification'] == 'internal' for w in welds),
                            outgoing_spotwelds=sum(w['classification'] == 'outgoing' for w in welds),
                            external_nodes=len(attachments['external_node_ids'])),
                declarations=dict(parts=_table(declarations, 'part', 'part_id'),
                                  sections=_table(declarations, 'section', 'section_id'),
                                  materials=_table(declarations, 'material', 'material_id'),
                                  curves=_table(declarations, 'hardening_curve', 'curve_id'), units=asdict(seed.units)),
                donor_policy=dict(DONOR_POLICY), geometry=asdict(geometry), attachments=attachments,
                parent_bindings=compile_parent_bindings(declarations, geometry),
                binding_order=dict(nodes='ascending source NID', parts='ascending source PID',
                                   parents='part order, then original canonical record order within each part',
                                   families='same parent traversal, separate zero-based QEPH and T3 counters',
                                   tables='ascending source material, section and curve IDs'),
                boundary=boundary,
                native_mass_ledger=dict(status='pending shell startup', additional_mass_assigned=False,
                                        selected_parts_complete=True, frontier_only_nodes_are_physical_owner_nodes=False),
                generator_sources=generator)
    if allow_linear:
        from .assembly_law44 import material_table
        from .law44_declarations import LINEAR_LAW44_POLICY
        result['declarations']['materials'] = material_table(declarations)
        result['law44_policy'] = dict(LINEAR_LAW44_POLICY)
    return result


def write_assembly_report(path, report):
    data = (json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + '\n').encode('utf-8')
    require(len(data) <= MAX_REPORT_BYTES, 'assembly report byte cap exceeded')
    with Path(path).open('xb') as stream:
        stream.write(data)
        stream.flush()
