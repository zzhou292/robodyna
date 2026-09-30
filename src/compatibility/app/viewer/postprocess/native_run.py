"""Native closure precheck. The C++ exact replay remains full archive authority."""
import math
import re

SCHEMA = 'robo_dyna.native_shell_impact_run.v1'
FORECAST_SCHEMA = 'robo_dyna.native_shell_impact_forecast.v1'
NATIVE_HISTORY_PAYLOADS = frozenset(('native_type25_accepted_base_force_history_v1',
    'fixed_main_type25_accepted_base_force_history_v1'))
STOP_KINDS = frozenset(('completed', 'requested_stop', 'diagnostic_interval_limit',
    'elapsed_limit', 'startup_failure', 'physics_rejected', 'archive_failure',
    'capture_failure', 'observer_failure'))


def _count(value, positive=False):
    if type(value) is not int or value < int(positive):
        raise ValueError('Malformed native run count')
    return value


def _time(value, positive=False):
    if type(value) not in (float, int) or not math.isfinite(value) or value < 0 or (positive and value == 0):
        raise ValueError('Malformed native run time')
    return value


def _record(root, record, expected_name, sha256):
    if not isinstance(record, dict) or set(record) != {'file', 'bytes', 'sha256'} or record['file'] != expected_name:
        raise ValueError('Native run record path differs from the closed output contract')
    count = _count(record['bytes'], True)
    digest = record['sha256']
    if not isinstance(digest, str) or re.fullmatch('[0-9a-f]{64}', digest) is None:
        raise ValueError('Native run record hash is malformed')
    path = root / expected_name
    if path.stat().st_size != count or sha256(path) != digest:
        raise ValueError('Native run record changed after closure')
    return digest


def closed_native_run(run, report):
    # Lazy import avoids a second file/hash implementation and circular setup.
    from .lifecycle import read_json, sha256
    if type(report.get('exit_code')) is not int or report['exit_code'] not in (0, 2):
        raise ValueError('Native guard exit code is not a normal typed result')
    summary = read_json(run / 'summary.json')
    if (summary.get('schema') != SCHEMA
            or summary.get('scope') != 'accepted_visualization_not_vehicle_delivery_or_restart'
            or summary.get('caption') != 'GPU shell-impact coupon'
            or summary.get('valid_closed_archive') is not True
            or summary.get('output_error') != ''):
        raise ValueError('Native run did not publish its declared closed archive')
    forecast = summary['forecast']
    if (forecast.get('schema') != FORECAST_SCHEMA
            or forecast.get('case') != 'declared_gpu_shell_impact_coupon'
            or forecast.get('scope') != 'capacity_bounds_not_stability_or_completed_physics'):
        raise ValueError('Native run forecast profile differs')
    run_id = _count(forecast['run_id'], True)
    planned = _count(forecast['planned_intervals'], True)
    samples = _count(forecast['samples'], True)
    if not 2 <= samples <= min(1000, planned + 1):
        raise ValueError('Native run sample count is outside the declared horizon')
    _time(forecast['fixed_dt_s'], True)
    _time(forecast['descriptive_duration_s'], True)
    accepted = _count(summary['accepted_intervals'])
    time = _time(summary['actual_time_s'])
    if accepted > planned or type(summary.get('requested_steps_reached')) is not bool:
        raise ValueError('Native run completion count differs')
    if summary['requested_steps_reached'] != (accepted == planned):
        raise ValueError('Native run requested-step claim differs')
    if (not isinstance(summary.get('reason'), str) or not isinstance(summary.get('stop_kind'), str)
            or summary['stop_kind'] not in STOP_KINDS):
        raise ValueError('Native run stop metadata is malformed')
    viewer_hash = _record(run, summary['viewer_input'], 'viewer-input.json', sha256)
    archive_root = run / 'archive'
    manifest_hash = _record(archive_root, summary['archive_manifest'], 'manifest.json', sha256)
    archive = read_json(archive_root / 'manifest.json')
    _record(archive_root, archive['configuration'], 'configuration.json', sha256)
    configuration = read_json(archive_root / 'configuration.json')
    profile = configuration['profile']
    if (configuration.get('schema') != 'robo_dyna.physical_run_configuration.v3'
            or profile.get('schema') != 'robo_dyna.physical_observation_profile.v3'
            or profile.get('participants') != 'qeph,t3,native_type25'
            or profile.get('native_contact') not in NATIVE_HISTORY_PAYLOADS
            or _count(configuration['identity']['run'], True) != run_id
            or _count(configuration['intervals'], True) != planned
            or _count(configuration['samples'], True) != samples
            or _time(configuration['fixed_dt_s'], True) != forecast['fixed_dt_s']
            or _time(configuration['requested_duration_s'], True) != forecast['descriptive_duration_s']):
        raise ValueError('Native summary forecast and authenticated archive profile differ')
    _record(archive_root, archive['index'], 'frame-index.json', sha256)
    index = read_json(archive_root / 'frame-index.json')
    if (not isinstance(index['frames'], list) or not index['frames']
            or type(index.get('horizon_complete')) is not bool
            or _count(index['accepted_intervals']) != accepted
            or _count(index['planned_intervals'], True) != planned
            or _count(index['final']['epoch']) != accepted
            or _time(index['final']['time']) != time
            or _count(index['frames'][-1]['stamp']['epoch']) != accepted
            or _time(index['frames'][-1]['stamp']['time']) != time):
        raise ValueError('Native run summary and final archived sample differ')
    complete = index['horizon_complete']
    if (complete != (accepted == planned) or complete != (report['exit_code'] == 0)
            or complete != (summary['stop_kind'] == 'completed')
            or index['stop_reason'] != summary['reason']
            or (complete and summary['reason'])
            or (not complete and not summary['reason'])):
        raise ValueError('Native run closure and prefix claims differ')
    # Normalize only this in-memory postprocessing view. Keep native schema and
    # summary.json bytes intact; never fabricate a legacy run-summary.json.
    values = {**summary, 'valid_archive_manifest': True,
              'actual_completed_time_s': time, 'viewer_input_sha256': viewer_hash,
              'archive_manifest_sha256': manifest_hash,
              '_summary_file': 'summary.json', '_display_profile': summary['caption']}
    return values, index, viewer_hash
