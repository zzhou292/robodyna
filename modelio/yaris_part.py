"""Offline SHA-pinned archive composition of a requested declaration closure."""
from dataclasses import asdict
import json
from pathlib import Path
import zipfile

from ._legacy import file_sha256, require, sha256
from .declarations import UnitSystem
from .keyword_cards import compile_part_declarations
from .source_blocks import scan_declarations, MAX_SOURCE_BYTES

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT / 'models/yaris_coarse_v1l.json'
VEHICLE = 'yaris-coarse-v1l.key'
DOCUMENTATION = {
    'material': 'https://help.altair.com/hwsolvers/rad/topics/solvers/rad/mat_024_piecewise_linear_plasticity_lsdyna_r.htm',
    'section': 'https://help.altair.com/hwsolvers/rad/topics/solvers/rad/section_shell_lsdyna_r.htm',
    'curve': 'https://help.altair.com/hwsolvers/rad/topics/solvers/rad/define_curve_lsdyna_r.htm',
    'part': 'https://help.altair.com/hwsolvers/rad/topics/solvers/rad/part_lsdyna_r.htm',
}


def declaration_report(declarations):
    material = declarations.material
    return {
        'schema': 'robo-dyna.source-part-declarations.v1',
        'scope': 'Selected PART/SECTION_SHELL/MAT024/DEFINE_CURVE closure; no geometry or dynamics',
        'simulation_ready': False,
        'source_declarations_validated': True,
        'geometry_modified': False,
        'source_geometry_validated_by_this_compiler': False,
        'declarations': asdict(declarations),
        'interpretation': {
            'hardening_source': 'positive LCSS references a stress-versus-plastic-strain curve',
            'supplied_SIGY_and_ETAN': 'preserved as supplied; ignored by this LCSS branch',
            'curve_abscissa': 'dimensionless plastic strain',
            'curve_ordinate': 'yield stress, converted from source units to Pa',
            'rate_type': 'total_strain_rate' if material.rate_type == 0 else 'unresolved_blank',
            'rate_parameters': 'C and P declared; no rate-dependent stress update or donor equivalence asserted',
            'donor_reference': 'Altair documents LAW36 for this curve branch; LAW44 is not assumed equivalent',
            'source_ELFORM': 'source formulation code retained; no mapping to the current TL shell is admitted',
            'blank_fields': 'None means absent in the source, never an implicit numerical zero',
            'curve_defaults': 'Only identity scales/offsets have documented defaults; each use is recorded',
        },
        'pending_before_simulation': [
            'Source shell formulation, native triangles, warpage and section/control defaults',
            'Material history, plasticity, rate integration, failure and deformation qualification',
            'Attachment closure and interaction with the surrounding vehicle',
            'Source geometry, thickness-based mass, center of mass and inertia ledger',
            'Contact, boundary/loading conditions and stable coupled stepping admission',
        ],
        'interpretation_references': {
            'urls': DOCUMENTATION,
            'reviewed_utc_date': '2026-09-09',
            'scope': 'Primary Altair LS-DYNA input-interface documentation; not complete LS-DYNA keyword equivalence',
        },
    }


def _member(archive, name, cap):
    require(archive.namelist().count(name) == 1, f'missing or duplicate archive member {name}')
    info = archive.getinfo(name)
    require(info.file_size <= cap and not info.is_dir(), f'oversized or invalid source member {name}')
    return info


def _compile_archive_part(archive_path, part_id=2000157):
    """Return a fully staged report. No output files or physical state are made."""
    require(isinstance(part_id, int) and not isinstance(part_id, bool) and 0 < part_id < 2**63,
            'part ID must be a positive signed-64-bit integer')
    archive_path = Path(archive_path)
    require(archive_path.is_file() and archive_path.stat().st_size <= MAX_SOURCE_BYTES,
            'source archive is missing or exceeds byte cap')
    reference_bytes = REFERENCE.read_bytes()
    reference = json.loads(reference_bytes)
    archive_hash = file_sha256(archive_path)
    require(archive_hash == reference['archive_sha256'], 'archive SHA256 mismatch')
    prefix = reference['archive_member_prefix']
    with zipfile.ZipFile(archive_path) as archive:
        _member(archive, prefix + VEHICLE, MAX_SOURCE_BYTES)
        _member(archive, prefix + 'README.md', 64 * 1024)
        readme = archive.read(prefix + 'README.md')
        readme_text = readme.decode('ascii')
        evidence = ('Mass **t**: metric ton (1,000 kg)', 'Length **mm**: millimeter',
                    'Force **N**: newton', 'Time **s**: second')
        require(all(value in readme_text for value in evidence), 'pinned README does not declare expected units')
        units = UnitSystem('t', 'mm', 's', 1000.0, .001, 1.0)
        with archive.open(prefix + VEHICLE) as stream:
            index = scan_declarations(stream, VEHICLE)
        require(index.sha256 == reference['files'][VEHICLE]['sha256'], 'vehicle source SHA256 mismatch')
    declarations = compile_part_declarations(index, part_id, units)
    report = declaration_report(declarations)
    report['source'] = {
        'model': reference['model'], 'archive_sha256': archive_hash,
        'archive_member': prefix + VEHICLE, 'member_sha256': index.sha256,
        'member_bytes': index.source_bytes, 'scanned_keyword_blocks': index.keyword_count,
        'all_other_entities': 'outside selected declaration closure; not interpreted or admitted',
        'unit_authority': {'member': prefix + 'README.md', 'sha256': sha256(readme),
                           'raw_text': readme_text, 'unit_declarations': evidence},
        'pinned_reference_sha256': sha256(reference_bytes),
    }
    sources = [Path(__file__).parent / name for name in
               ('_legacy.py', 'source_blocks.py', 'declarations.py', 'keyword_cards.py', 'yaris_part.py')]
    sources += [ROOT / 'tools' / name for name in
                ('compile_yaris_part.py', 'import_yaris_vehicle.py', 'import_yaris_wall.py')]
    report['generator_sources'] = {str(path.relative_to(ROOT)): file_sha256(path) for path in sources}
    return declarations, report


def compile_archive_part(archive_path, part_id=2000157):
    """Existing E1 entry point and report schema remain unchanged."""
    return _compile_archive_part(archive_path, part_id)[1]


def compile_archive_readiness(archive_path, asset_dir, quadrature_path, part_id=2000157):
    """Opt-in E2a composition; no attachment or physical-state admission."""
    from .canonical_geometry import load_part_geometry, read_json
    from .shell_mass import audit_shell_surface, load_quadrature

    declarations, e1 = _compile_archive_part(archive_path, part_id)
    reference = read_json(REFERENCE)
    geometry = load_part_geometry(asset_dir, archive_path, declarations, reference)
    quadrature = load_quadrature(quadrature_path)
    mass = audit_shell_surface(geometry, declarations, quadrature)
    attachments = dict(typed_closure_qualified=False, attachment_mechanics_implemented=False,
                       tied_pairing_qualified=False, cut_boundary_supplied=False,
                       scope='Read-only pinned source inventory; E2b typed closure remains pending')
    if part_id == 2000157 and reference['archive_sha256'] == \
            'aff8194c456726a678d6cc11f644316ca70f3d9b37c4db622726b7b2985b0451':
        # Audited inventory facts only, bound to this exact source archive. These
        # are not newly parsed constraints and never become mechanics inputs.
        attachments.update(
            known_nodal_rigid_groups=[dict(rigid_body_id=2200909 + i, node_set_id=2200909 + i,
                                          source_file=VEHICLE, source_line=24808 + 9 * i,
                                          node_set_line=24811 + 9 * i) for i in range(6)],
            known_group_nodes=dict(total_unique=76, in_selected_part=20, external=56),
            known_external_part_ids=[2000119, 2000120, 2000145, 2000165, 2000260],
            known_tied_scope=dict(keyword='*CONTACT_TIED_SHELL_EDGE_TO_SURFACE', source_line=36,
                                 source_file=VEHICLE, slave_part_set=2000002, master_part_set=2000001,
                                 selected_part_is_master_set_member=True,
                                 actual_pairing='unresolved; membership is not an attachment pair'),
            unresolved=['nodal-rigid optional/default mechanics', 'neighbor transitive closure',
                        'tied projection, tolerance and default semantics', 'added mass and setup transformations'])
    else:
        attachments['unresolved'] = ['all source attachment and interaction scopes for this selection']
    report = dict(schema='robo-dyna.source-part-readiness.v1', simulation_ready=False,
                  source_mass_equivalence_qualified=False, source_declarations=e1,
                  geometry=asdict(geometry), surface_mass_audit=mass, attachments=attachments,
                  selected_geometry_source_verified=True, geometry_modified=False, mechanics_capacity_changed=False)
    paths = [Path(__file__), Path(__file__).with_name('canonical_geometry.py'),
             Path(__file__).with_name('shell_mass.py'), ROOT / 'tools/compile_yaris_part.py']
    report['readiness_generator_sources'] = {str(p.relative_to(ROOT)): file_sha256(p) for p in paths}
    return report


def compile_archive_attachment_readiness(archive_path, asset_dir, quadrature_path, part_id=2000157):
    """Explicit E2b v2 envelope; the E2a v1 API/inventory remains available.

    Additional typed source coverage is separate from the earlier pinned facts.
    This never upgrades the legacy inventory or source mechanics qualification.
    """
    from .canonical_geometry import load_part_geometry, read_json
    from .attachments import compile_attachment_scope
    from .source_blocks import scan_metadata, DECLARATION_FAMILIES, ATTACHMENT_FAMILIES

    e2a = compile_archive_readiness(archive_path, asset_dir, quadrature_path, part_id)
    reference = read_json(REFERENCE)
    member = reference['archive_member_prefix'] + VEHICLE
    with zipfile.ZipFile(archive_path) as archive:
        _member(archive, member, MAX_SOURCE_BYTES)
        with archive.open(member) as stream:
            index = scan_metadata(stream, VEHICLE, families=DECLARATION_FAMILIES | ATTACHMENT_FAMILIES)
    require(index.sha256 == reference['files'][VEHICLE]['sha256'], 'attachment source SHA256 mismatch')
    # Reuse the typed declaration parser and source-authenticated geometry owner.
    # No report dictionary is treated as a mutable substitute for typed input.
    declarations = compile_part_declarations(index, part_id, UnitSystem('t', 'mm', 's', 1000., .001, 1.))
    geometry = load_part_geometry(asset_dir, archive_path, declarations, reference)
    attachment = compile_attachment_scope(index, geometry, asset_dir, archive_path, reference)
    paths = [Path(__file__).parent / name for name in
             ('_legacy.py', 'source_blocks.py', 'attachment_cards.py', 'attachments.py', 'canonical_incidence.py',
              'canonical_geometry.py', 'yaris_part.py')]
    paths += [ROOT / 'tools' / name for name in ('compile_yaris_part.py', 'import_yaris_vehicle.py', 'import_yaris_wall.py')]
    return dict(schema='robo-dyna.source-part-readiness.v2',
                scope='E2a plus typed original-include one-hop attachment inventory; no assembled closure or mechanics',
                simulation_ready=False, geometry_modified=False, mechanics_capacity_changed=False,
                source_mass_equivalence_qualified=False, full_attachment_closure_qualified=False,
                e2a_readiness=e2a, typed_attachment_scope=asdict(attachment),
                attachment_generator_sources={str(p.relative_to(ROOT)): file_sha256(p) for p in paths})


def write_report(path, report):
    """Create-only publication after source validation; no overwrite or mkdir."""
    data = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + '\n'
    require(len(data.encode('utf-8')) <= 1024 * 1024, 'declaration report exceeds 1 MiB cap')
    with Path(path).open('x', encoding='utf-8') as stream:
        stream.write(data)
        stream.flush()
