"""Full V5 closure precheck; exact C++ replay owns complete record validation."""
from .native_run import _count, _record, _time

SCHEMA = 'robo_dyna.native_vehicle_contact_run.v1'
PARTICIPANTS = 'qeph,t3,qbat,type25,type13,solids,type45,beam18,native_type25_group'
# Numeric values are the existing vehicle_run::StopKind contract. Output and
# startup failures cannot claim a normal closed archive.
CLOSED_STOPS = frozenset((0, 1, 2, 3, 5, 8))


def closed_native_vehicle_run(run, report):
    from .lifecycle import read_json, sha256
    if type(report.get('exit_code')) is not int or report['exit_code'] not in (0, 2):
        raise ValueError('Vehicle producer guard has no normal typed completion')
    summary = read_json(run / 'summary.json')
    if (summary.get('schema') != SCHEMA
            or summary.get('physical_profile') != 'selected_vehicle_supports_v5_with_declared_finite_mesh_wall'
            or summary.get('contact_profile') != 'source_type25_self_and_all_retained_nodes_to_fixed_mesh'
            or summary.get('initial_state') != 'source_produced_starter_history_and_final_type2_removals'
            or summary.get('visualization_only_not_restart') is not True
            or summary.get('session_initialized') is not True
            or summary.get('valid_closed_archive') is not True
            or summary.get('viewer_error', '') != ''):
        raise ValueError('Native vehicle source/output profile is not a closed accepted archive')
    planned = _count(summary['planned_intervals'], True)
    accepted = _count(summary['accepted_intervals'])
    time = _time(summary['actual_time_s'])
    dt = _time(summary['fixed_dt_s'], True)
    duration = _time(summary['requested_duration_s'], True)
    kind = summary['stop_kind']
    reason = summary['stop_reason']
    if (type(kind) is not int or kind not in CLOSED_STOPS or not isinstance(reason, str)
            or len(reason) > 4096 or type(summary.get('horizon_complete')) is not bool
            or accepted > planned or (accepted == 0) != (time == 0)):
        raise ValueError('Native vehicle accepted count/time/stop metadata differs')
    viewer_hash = _record(run, summary['viewer_input'], 'viewer-input.json', sha256)
    archive_root = run / 'archive'
    manifest_hash = _record(archive_root, summary['archive_manifest'], 'manifest.json', sha256)
    viewer = read_json(run / 'viewer-input.json')
    if (viewer.get('schema') != 'robo_dyna.physical_viewer_input.v1'
            or viewer.get('archive_directory') != 'archive'
            or viewer.get('manifest') != summary['archive_manifest']):
        raise ValueError('Native vehicle viewer input does not bind the closed manifest')
    archive = read_json(archive_root / 'manifest.json')
    if archive.get('schema') != 'robo_dyna.physical_accepted_run.v2':
        raise ValueError('Native vehicle archive lacks the explicit environment schema')
    _record(archive_root, archive['configuration'], 'configuration.json', sha256)
    configuration = read_json(archive_root / 'configuration.json')
    profile = configuration['profile']
    if (configuration.get('schema') != 'robo_dyna.physical_run_configuration.v5'
            or configuration.get('environment_wall') != 'native_declared_fixed_elastic_wall_v1'
            or profile.get('schema') != 'robo_dyna.physical_observation_profile.v4'
            or profile.get('purpose') != 'selected_physical_model_accepted_visualization_not_restart'
            or profile.get('participants') != PARTICIPANTS
            or profile.get('native_group') != 'declared_order_common_owner_accepted_publications_v1'
            or 'native_contact' in profile or 'self_contact' in profile
            or _count(configuration['intervals'], True) != planned
            or _time(configuration['fixed_dt_s'], True) != dt
            or _time(configuration['requested_duration_s'], True) != duration):
        raise ValueError('Native vehicle summary differs from its authenticated archive profile/horizon')
    samples = _count(configuration['samples'], True)
    if not 2 <= samples <= min(1000, planned + 1):
        raise ValueError('Native vehicle sample plan is outside its accepted horizon')
    _record(archive_root, archive['index'], 'frame-index.json', sha256)
    index = read_json(archive_root / 'frame-index.json')
    if (index.get('schema') != 'robo_dyna.physical_accepted_index.v1'
            or not isinstance(index.get('frames'), list) or not index['frames']
            or type(index.get('horizon_complete')) is not bool
            or _count(index['accepted_intervals']) != accepted
            or _count(index['planned_intervals'], True) != planned
            or _count(index['final']['epoch']) != accepted
            or _time(index['final']['time']) != time
            or _count(index['frames'][0]['stamp']['epoch']) != 0
            or _time(index['frames'][0]['stamp']['time']) != 0
            or _count(index['frames'][-1]['stamp']['epoch']) != accepted
            or _time(index['frames'][-1]['stamp']['time']) != time):
        raise ValueError('Native vehicle summary differs from the archived initial/final samples')
    complete = index['horizon_complete']
    if (complete != (accepted == planned) or complete != summary['horizon_complete']
            or complete != (kind == 0) or complete != (report['exit_code'] == 0)
            or index['stop_reason'] != reason or (complete and reason) or (not complete and not reason)):
        raise ValueError('Native vehicle completion and diagnostic prefix claims differ')
    interfaces = summary['interfaces']
    if not isinstance(interfaces, list) or len(interfaces) != 2:
        raise ValueError('Native vehicle summary must retain both declared interfaces')
    roles, ids, previous = set(), set(), 0
    for entry in interfaces:
        role = entry['role']
        source_id = _count(entry['native_id'], True)
        ordinal = _count(entry['native_storage_ordinal'], True)
        if (role not in ('self', 'mesh_wall') or role in roles or source_id in ids
                or ordinal <= previous or _count(entry['accepted_intervals']) != accepted
                or _count(entry['active_force_intervals']) > accepted
                or entry.get('initialization_available') is not True):
            raise ValueError('Native vehicle accepted interface counters/order differ')
        roles.add(role)
        ids.add(source_id)
        previous = ordinal
    # Only an in-memory operator view is normalized. No legacy summary, native
    # profile, owner identity or archive bytes are synthesized or rewritten.
    values = {**summary, 'valid_archive_manifest': True, 'reason': reason,
              'actual_completed_time_s': time, 'viewer_input_sha256': viewer_hash,
              'archive_manifest_sha256': manifest_hash, '_summary_file': 'summary.json',
              '_display_profile': 'V5 vehicle + finite mesh wall'}
    return values, index, viewer_hash
