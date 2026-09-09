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


def compile_archive_part(archive_path, part_id=2000157):
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
    return report


def write_report(path, report):
    """Create-only publication after source validation; no overwrite or mkdir."""
    data = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + '\n'
    require(len(data.encode('utf-8')) <= 1024 * 1024, 'declaration report exceeds 1 MiB cap')
    with Path(path).open('x', encoding='utf-8') as stream:
        stream.write(data)
        stream.flush()
