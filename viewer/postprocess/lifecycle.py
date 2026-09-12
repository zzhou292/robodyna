"""Read-only simulation completion checks and durable postprocessing status."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def write_status(directory, **values):
    values['updated_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
    path = Path(directory) / 'status.json'
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(values, indent=2) + '\n')
    temporary.replace(path)
    print(json.dumps(values), flush=True)


def guard_alive(launch, proc=Path('/proc')):
    """A reused PID is not the launched guard. Zombies cannot keep running."""
    try:
        fields = (proc / str(launch['guard_pid']) / 'stat').read_text().rsplit(')', 1)[1].split()
    except FileNotFoundError:
        return False
    return (fields[0] != 'Z' and int(fields[19]) == launch['guard_start_ticks']
            and int(fields[3]) == launch['guard_sid'])


def closed_run(launch):
    """Require normal archive publication, including a valid diagnostic prefix."""
    report = read_json(launch['guard_report'])
    if report.get('process_scope', {}).get('cleanup') != 'complete':
        raise ValueError('Simulation guard did not complete owned-session cleanup')
    if report.get('exit_code') not in (0, 2):
        raise ValueError('Simulation did not close normally; inspect its guard report')
    run = Path(launch['output'])
    summary = read_json(run / 'run-summary.json')
    if summary.get('valid_archive_manifest') is not True:
        raise ValueError('Simulation has no valid closed archive')
    receipt = run / 'viewer-input.json'
    manifest = run / 'archive/manifest.json'
    if sha256(receipt) != summary['viewer_input_sha256']:
        raise ValueError('Viewer receipt differs from the completed run summary')
    if sha256(manifest) != summary['archive_manifest_sha256']:
        raise ValueError('Archive manifest differs from the completed run summary')
    archive = read_json(manifest)
    index_path = run / 'archive/frame-index.json'
    if sha256(index_path) != archive['index']['sha256']:
        raise ValueError('Archive frame index differs from its manifest')
    index = read_json(index_path)
    if index['accepted_intervals'] != summary['accepted_intervals'] or not index['frames']:
        raise ValueError('Archive accepted count differs or contains no saved frames')
    return summary, index, sha256(receipt)


def notify(title, message, environment):
    """Notify this workstation's desktop; failure never invalidates a video."""
    try:
        result = subprocess.run(['/usr/bin/notify-send', '--app-name=robo-dyna',
                                 title, message], env={**os.environ, **environment},
                                capture_output=True, text=True, timeout=10)
        return {'sent': result.returncode == 0, 'detail': result.stderr.strip()}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'sent': False, 'detail': str(error)}


def wait_for_run(launch, timeout, poll_seconds=30):
    deadline = time.monotonic() + timeout
    while guard_alive(launch):
        if time.monotonic() >= deadline:
            raise TimeoutError('Postprocessing wait expired; simulation was left untouched')
        time.sleep(min(poll_seconds, max(0, deadline - time.monotonic())))
    # The guard publishes its final receipt before exiting. No partial recovery
    # or restart is inferred if publication is missing.
    return closed_run(launch)
