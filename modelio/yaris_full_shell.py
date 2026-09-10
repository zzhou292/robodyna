"""Reusable full-shell scope compiler; authenticated geometry is not dynamics."""
from dataclasses import asdict
import json
from pathlib import Path

from ._legacy import file_sha256,require
from .full_shell_source import load_full_shell_source
from .full_shell_coverage import coverage,tire_exclusions
from .full_shell_declarations import declaration_coverage
from .full_shell_connections import connection_coverage

MAX_SCOPE_BYTES=32*1024*1024


def compile_full_shell_scope(archive_path,assets,tire_policy='retain_all'):
    require(tire_policy in ('retain_all','omit_original_tire_shells'),'unknown full-shell tire policy')
    source=load_full_shell_source(archive_path,assets)
    excluded=tire_exclusions(source.geometry,tire_policy)
    counts,rows,parts,nodes=coverage(source.geometry,excluded)
    declarations=declaration_coverage(source,rows,parts,excluded)
    connections=connection_coverage(source,parts,nodes,rows)
    root=Path(__file__).resolve().parents[1]
    names=['modelio/'+n for n in (
        'full_shell_source.py','full_shell_coverage.py','full_shell_declarations.py',
        'full_shell_connections.py','full_shell_materials.py','yaris_full_shell.py',
        '_legacy.py','canonical_geometry.py','canonical_incidence.py','source_blocks.py',
        'spotweld_cards.py','attachment_cards.py','yaris_part.py','keyword_cards.py',
        'assembly_declarations.py','declarations.py','law44_declarations.py')]
    names += ['tools/import_yaris_vehicle.py','tools/import_yaris_wall.py',
              'tools/compile_yaris_full_shell.py','models/yaris_coarse_v1l.json']
    return dict(schema='robo-dyna.full-shell-scope.v1',simulation_ready=False,
        geometry_modified=False,source_mass_equivalence_qualified=False,
        source=dict(source.authority,
            all_other_entities='Complete original non-shell and auxiliary source inventories retained; unsupported mechanics and mass/load-path closure are not admitted',
            scope='Complete original vehicle-include shell selection; auxiliary includes retained as unresolved source obligations'),
        units=asdict(source.units),tire_policy=tire_policy,excluded_tire_shell_part_ids=sorted(excluded),
        selected_shell_part_ids=sorted(parts),coverage=counts,declarations=declarations,connections=connections,
        canonical=dict(manifest_sha256=source.manifest_sha256,arrays=source.manifest['arrays'],
            selection='all original shell records whose PID is in selected_shell_part_ids; all their source nodes',
            source_records_independently_verified=True),
        source_keyword_census={n:s['keyword_counts'] for n,s in source.source_files.items()},
        non_shell_mass_policy='Omitted solid/beam/discrete/added-mass contributions are unknown, not zero; no mass substitute has been applied',
        runtime_admission='blocked pending full source mechanics, connection/load-path and independent capacity admission',
        generator_sources={n:file_sha256(root/n) for n in names})


def write_full_shell_scope(path,report):
    require(report.get('schema')=='robo-dyna.full-shell-scope.v1' and report.get('simulation_ready') is False,
            'invalid full-shell scope report')
    data=(json.dumps(report,indent=2,sort_keys=True,allow_nan=False)+'\n').encode()
    require(len(data)<=MAX_SCOPE_BYTES,'full-shell scope report byte cap exceeded')
    with Path(path).open('xb') as output:output.write(data)
