#!/usr/bin/env python3
"""Run one development command with bounded CPU use and monitored memory.

This is a cooperative workstation guard, not a GPU memory allocator or cgroup.
Children inherit CPU affinity and thread limits. Memory limits are sampled;
the owned session is terminated if a limit is exceeded. An explicit stop
file permits a short controller grace for GPU growth alone; hard limits remain
active. No other user's processes are touched. Use the same lock path for all
builds/tests in a session. Retained telemetry is bounded; every live sample
still participates in limit checks and the whole-run sampled RSS peak.
"""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

if __package__:
    from .bounded_history import SampleHistory
    from .bounded_gpu import GpuProcessDiagnostics
    from .bounded_session import OwnedSession, preflight as session_preflight
    from .bounded_stop import CooperativeStop, GPU_GROWTH_REASON
else:
    from bounded_history import SampleHistory
    from bounded_gpu import GpuProcessDiagnostics
    from bounded_session import OwnedSession, preflight as session_preflight
    from bounded_stop import CooperativeStop, GPU_GROWTH_REASON

MIB = 1024 * 1024
GIB = 1024 * MIB


class GuardInterrupted(BaseException):
    """Do not let optional diagnostic Exception handlers consume SIGTERM."""



def memory_info():
    values = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        key, value, *_ = line.split()
        values[key.rstrip(':')] = int(value) * 1024
    return values


def gpu_info(index):
    result = subprocess.run(
        ['nvidia-smi', '-i', str(index),
         '--query-gpu=memory.total,memory.used,memory.free,utilization.gpu',
         '--format=csv,noheader,nounits'],
        capture_output=True, text=True, timeout=5, check=True)
    total, used, free, utilization = [int(v.strip()) for v in result.stdout.strip().split(',')]
    return dict(total_bytes=total * MIB, used_bytes=used * MIB,
                free_bytes=free * MIB, utilization_percent=utilization)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--lock', type=Path)
    parser.add_argument('--cpus', type=int, default=2)
    parser.add_argument('--min-available-gib', type=float, default=32)
    parser.add_argument('--max-rss-gib', type=float, default=8)
    parser.add_argument('--timeout', type=float, default=600)
    parser.add_argument('--gpu', type=int, help='require and monitor this physical GPU')
    parser.add_argument('--gpu-process-diagnostics', action='store_true',
                        help='record bounded compute-process telemetry; never changes GPU guard limits')
    parser.add_argument('--min-gpu-free-gib', type=float, default=8)
    parser.add_argument('--max-gpu-growth-gib', type=float, default=4)
    parser.add_argument('--cooperative-stop-file', type=Path,
                        help='on GPU growth only, create this fresh controller stop file before killing')
    parser.add_argument('--cooperative-stop-grace-seconds', type=float,
                        help='positive finite grace after that request (default 30 seconds)')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command or args.cpus < 1 or args.timeout <= 0 or any(
            value < 0 for value in [args.min_available_gib, args.max_rss_gib,
                                   args.min_gpu_free_gib, args.max_gpu_growth_gib]):
        parser.error('provide a command and positive CPU/timeout, nonnegative memory limits')
    if args.gpu_process_diagnostics and args.gpu is None:
        parser.error('--gpu-process-diagnostics requires --gpu')
    if args.cooperative_stop_file is None and args.cooperative_stop_grace_seconds is not None:
        parser.error('--cooperative-stop-grace-seconds requires --cooperative-stop-file')
    grace_seconds = args.cooperative_stop_grace_seconds
    if grace_seconds is None:
        grace_seconds = 30.0
    if args.cooperative_stop_file is not None and (
            args.gpu is None or not math.isfinite(grace_seconds) or grace_seconds <= 0):
        parser.error('cooperative stop requires --gpu and a finite positive grace')
    args.report.parent.mkdir(parents=True, exist_ok=True)
    lock_path = args.lock or args.report.parent / 'workstation.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    report = dict(command=command, limits=dict(cpus=args.cpus,
                  min_available_gib=args.min_available_gib, max_rss_gib=args.max_rss_gib,
                  timeout_seconds=args.timeout, gpu=args.gpu,
                  min_gpu_free_gib=args.min_gpu_free_gib,
                  max_gpu_growth_gib=args.max_gpu_growth_gib), samples=[], status='preflight')
    history = SampleHistory()
    cooperative = None
    if args.cooperative_stop_file is not None:
        cooperative = CooperativeStop(args.cooperative_stop_file, grace_seconds)
        report['cooperative_stop'] = cooperative.record
    process = None
    session = None
    lock = None
    started = time.monotonic()
    gpu_processes = GpuProcessDiagnostics(args.gpu, started) if args.gpu_process_diagnostics else None
    def interrupted(signum, _frame):
        raise GuardInterrupted('runner interrupted by signal ' + str(signum))
    previous_term = signal.signal(signal.SIGTERM, interrupted)
    try:
        lock = lock_path.open('a')
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if cooperative is not None:
            cooperative.preflight(args.report, lock_path)
        session_preflight()
        available = sorted(os.sched_getaffinity(0))
        affinity = available[:min(args.cpus, len(available))]
        mem = memory_info()
        report['initial_memory'] = mem
        if mem['MemAvailable'] < args.min_available_gib * GIB:
            raise RuntimeError('insufficient available host RAM before launch')
        initial_gpu = gpu_info(args.gpu) if args.gpu is not None else None
        if initial_gpu and initial_gpu['free_bytes'] < args.min_gpu_free_gib * GIB:
            raise RuntimeError('insufficient free GPU memory before launch')
        report['initial_gpu'] = initial_gpu
        report['cpu_affinity'] = affinity
        env = os.environ.copy()
        env.update(OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1',
                   NUMEXPR_NUM_THREADS='1', CMAKE_BUILD_PARALLEL_LEVEL=str(len(affinity)),
                   CTEST_PARALLEL_LEVEL='1')
        if args.gpu is not None:
            env['CUDA_DEVICE_ORDER'] = 'PCI_BUS_ID'
            env['CUDA_VISIBLE_DEVICES'] = str(args.gpu)

        def prepare_child():
            os.sched_setaffinity(0, affinity)
            os.nice(10)

        print('bounded:', json.dumps(dict(command=command, cpus=affinity,
                                         gpu=args.gpu)), flush=True)
        process = subprocess.Popen(command, env=env, start_new_session=True,
                                   preexec_fn=prepare_child)
        session = OwnedSession(process)
        report['process_scope'] = session.record
        previous_gpu = initial_gpu
        previous_gpu_poll_elapsed = 0.0
        next_gpu_poll = started
        while True:
            now = time.monotonic()
            mem = memory_info()
            usage = session.usage()
            gpu_polled = False
            if args.gpu is not None and now >= next_gpu_poll:
                previous_gpu = gpu_info(args.gpu)
                previous_gpu_poll_elapsed = now - started
                gpu_polled = True
                next_gpu_poll = now + 2
            sample = dict(elapsed_seconds=round(now - started, 3),
                          available_bytes=mem['MemAvailable'],
                          free_bytes=mem['MemFree'], **usage, gpu=previous_gpu)
            history.append(sample)
            reason = None
            if mem['MemAvailable'] < args.min_available_gib * GIB:
                reason = 'available RAM fell below reserve'
            elif usage['rss_bytes'] > args.max_rss_gib * GIB:
                reason = 'job RSS exceeded budget'
            elif previous_gpu and previous_gpu['free_bytes'] < args.min_gpu_free_gib * GIB:
                reason = 'free GPU memory fell below reserve'
            elif previous_gpu and previous_gpu['used_bytes'] - initial_gpu['used_bytes'] > args.max_gpu_growth_gib * GIB:
                reason = GPU_GROWTH_REASON
            elif now - started > args.timeout:
                reason = 'command timeout'
            if reason:
                if cooperative is None or reason != GPU_GROWTH_REASON:
                    raise RuntimeError(reason)
                # Timeout remains hard, including when growth has priority
                # in the original diagnostic ordering. A finished command
                # cannot be asked to finalize an accepted prefix.
                elapsed = time.monotonic() - started
                if elapsed > args.timeout:
                    raise RuntimeError('command timeout')
                if cooperative.deadline is None and session.exited():
                    raise RuntimeError(reason)
                cooperative.request(elapsed, sample)
            if cooperative is not None and cooperative.deadline is not None:
                elapsed = time.monotonic() - started
                if elapsed > args.timeout:
                    raise RuntimeError('command timeout')
                if cooperative.expired(elapsed):
                    raise RuntimeError('cooperative stop grace expired')
            if gpu_processes is not None and gpu_polled:
                try:
                    gpu_processes.sample(session, 'poll', previous_gpu, previous_gpu_poll_elapsed)
                except Exception as diagnostic_error:
                    report['gpu_process_diagnostic_error'] = str(diagnostic_error)[:256]
            if session.exited():
                break
            time.sleep(0.25)
        # Do not leave subprocesses running after the command exits.
        session.stop()
        report['exit_code'] = process.returncode
        report['status'] = 'passed' if process.returncode == 0 else 'command_failed'
        if cooperative is not None and cooperative.deadline is not None:
            cooperative.exited(time.monotonic() - started, process.returncode)
            if process.returncode == 0:
                report['status'] = 'cooperatively_stopped'
            report['reason'] = GPU_GROWTH_REASON
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError, KeyboardInterrupt, GuardInterrupted) as error:
        if cooperative is not None:
            cooperative.forced(time.monotonic() - started, str(error))
        if process is not None:
            try:
                if session is not None:
                    # Snapshot the live process before cleanup destroys PID/GPU
                    # evidence. Whole-device reason and enforcement are unchanged.
                    growth_stop = (str(error) == GPU_GROWTH_REASON or
                                   (cooperative is not None and cooperative.deadline is not None))
                    try:
                        if gpu_processes is not None and growth_stop:
                            gpu_processes.sample(session, 'before_gpu_growth_stop',
                                                 previous_gpu, previous_gpu_poll_elapsed)
                    except (Exception, GuardInterrupted, KeyboardInterrupt) as diagnostic_error:
                        # The selected hard-stop reason is already authoritative.
                        report['gpu_process_diagnostic_error'] = str(diagnostic_error)[:256]
                    finally:
                        session.stop()
                else:
                    # Authentication itself failed. This direct child remains
                    # unreaped and owned, but no broader session is claimed.
                    process.kill()
                    process.wait(timeout=2)
                    report['cleanup_error'] = 'session authentication failed; only direct child stopped'
            except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as cleanup_error:
                report['cleanup_error'] = str(cleanup_error)
            if cooperative is not None and cooperative.record['outcome'] != 'not_requested':
                key = ('termination_failed_elapsed_seconds' if 'cleanup_error' in report
                       else 'termination_completed_elapsed_seconds')
                cooperative.record[key] = time.monotonic() - started
        report['status'] = 'blocked_or_stopped'
        report['reason'] = str(error)
        report['exit_code'] = 125
        print('bounded: stopped:', error, file=sys.stderr, flush=True)
    finally:
        signal.signal(signal.SIGTERM, previous_term)
        report['elapsed_seconds'] = round(time.monotonic() - started, 3)
        report['peak_sampled_rss_bytes'] = history.peak_rss_bytes
        report['samples'] = history.samples()
        report['sample_history'] = history.metadata()
        if gpu_processes is not None:
            try:
                report['gpu_process_diagnostics'] = gpu_processes.document()
            except Exception as diagnostic_error:
                report['gpu_process_diagnostics'] = dict(status='diagnostic_error',
                    error=str(diagnostic_error)[:256])
        try:
            args.report.write_text(json.dumps(report, indent=2) + '\n')
        finally:
            if lock is not None:
                lock.close()
    return report.get('exit_code', 125)


if __name__ == '__main__':
    sys.exit(main())
