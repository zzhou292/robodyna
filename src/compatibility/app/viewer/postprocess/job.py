"""Sequence existing archive replay, Chrono capture, and video tools after a run."""
import fcntl
import os
from pathlib import Path
import subprocess
import sys

from .lifecycle import notify, read_json, sha256, wait_for_run, write_status
from .replay_evidence import EXACT_REPLAY_TEST, require_exact_replay


def bounded_command(config, directory, label, command, gpu=False):
    report = directory / (label + '.guard.json')
    args = [sys.executable, '-B', config['guard'], '--report', str(report),
            '--lock', config['workstation_lock'], '--cpus', '2',
            '--min-available-gib', '32', '--max-rss-gib', '10', '--timeout', '1200']
    if gpu:
        args += ['--gpu', '0', '--min-gpu-free-gib', '8', '--max-gpu-growth-gib', '6']
    args += ['--', *map(str, command)]
    environment = {**os.environ, **config['environment'],
                   'PYTHONPATH': config['application'],
                   'ROBO_DYNA_PHYSICAL_REPLAY_INPUT': str(Path(config['run']) / 'viewer-input.json')}
    with (directory / (label + '.log')).open('x') as log:
        subprocess.run(args, cwd=config['workspace'], env=environment,
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    result = read_json(report)
    if result.get('status') != 'passed' or result.get('process_scope', {}).get('cleanup') != 'complete':
        raise ValueError(f'{label} did not pass its resource guard and cleanup')


def run_job(config):
    directory = Path(config['job_directory'])
    directory.mkdir(exist_ok=True)
    with (directory / 'job.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if (directory / 'status.json').exists():
            raise FileExistsError('Postprocessing already started; preserve its evidence')
        notifications = []
        videos = []
        try:
            launch = read_json(config['launch_receipt'])
            if Path(launch['output']).resolve() != Path(config['run']).resolve():
                raise ValueError('Launch receipt and postprocessing run differ')
            write_status(directory, stage='waiting_for_simulation', run=config['run'])
            summary, index, receipt_sha = wait_for_run(launch, config['wait_timeout_s'])
            count = summary['accepted_intervals']
            duration_ms = summary['actual_completed_time_s'] * 1000
            reached = count == config['requested_steps']
            label = f"{Path(config['run']).name} | {summary.get('_display_profile', 'physical simulation')}"
            message = f'{label}: {count:,} accepted steps, {duration_ms:.6g} ms saved. Rendering is starting.'
            notifications.append(notify('Simulation finished' if reached else 'Simulation stopped early',
                                        message, config['environment']))
            evidence = dict(accepted_steps=count, simulated_time_s=duration_ms / 1000,
                            requested_steps_reached=reached, saved_frames=len(index['frames']),
                            stop_reason=summary['reason'], viewer_receipt_sha256=receipt_sha,
                            run_summary_sha256=sha256(Path(config['run']) / summary.get('_summary_file', 'run-summary.json')),
                            summary_file=summary.get('_summary_file', 'run-summary.json'))
            write_status(directory, stage='verifying_replay', **evidence, notifications=notifications)
            for path, expected in config['pinned_files'].items():
                if sha256(path) != expected:
                    raise ValueError(f'Prepared tool changed before postprocessing: {path}')
            test_report = directory / 'archive-replay.xml'
            bounded_command(config, directory, 'archive-replay', [config['scene_checker'],
                '--gtest_filter=' + EXACT_REPLAY_TEST,
                '--gtest_output=xml:' + str(test_report)])
            require_exact_replay(test_report)
            for view in config['views']:
                name = view['name']
                capture = directory / (name + '-capture')
                output = directory / (name + '-video')
                write_status(directory, stage='rendering', view=name, **evidence,
                             notifications=notifications, videos=videos)
                bounded_command(config, directory, name + '-capture', [config['viewer'], config['run'],
                    '--capture', str(capture), '--capture-cap-gib', '2', '--color', 'part-id',
                    '--fps', '5', '--require-frames', str(len(index['frames'])),
                    '--receipt-sha256', receipt_sha, *view['camera_arguments']], gpu=True)
                bounded_command(config, directory, name + '-encode', [sys.executable, '-B', '-m',
                    'viewer.video.encode', str(capture), str(output), '--ffmpeg', config['ffmpeg'],
                    '--samples-per-second', '5', '--output-fps', '30'])
                movies = list(output.glob('*.mp4'))
                if len(movies) != 1:
                    raise ValueError('Encoder did not publish exactly one video')
                videos.append(str(movies[0]))
            notifications.append(notify('Simulation videos ready',
                f'{label}: {directory}', config['environment']))
            write_status(directory, stage='complete', **evidence,
                         videos=videos, notifications=notifications)
        except Exception as error:
            notifications.append(notify('Simulation postprocessing needs attention', str(error),
                                        config['environment']))
            write_status(directory, stage='failed', error=str(error), videos=videos,
                         notifications=notifications)
            raise
