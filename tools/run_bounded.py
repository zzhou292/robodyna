#!/usr/bin/env python3
"""Run one development command with bounded CPU use and monitored memory.

This is a cooperative workstation guard, not a GPU memory allocator or cgroup.
Children inherit CPU affinity and thread limits. Memory limits are sampled;
the owned process group is terminated if a limit is exceeded. No other user's
processes are touched. Use the same lock path for all builds/tests in a session.
"""

import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

MIB = 1024 * 1024
GIB = 1024 * MIB


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


def group_usage(pgid):
    """Sum RSS and live CPU ticks across this job's inherited process group."""
    rss, ticks, threads = 0, 0, 0
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            # comm can contain spaces and parentheses; fields after its final
            # ')' start at stat field 3. pgrp=5, utime=14, stime=15, rss=24.
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) != pgid:
                continue
            rss += int(fields[21]) * os.sysconf('SC_PAGE_SIZE')
            ticks += int(fields[11]) + int(fields[12])
            threads += int(fields[17])
        except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError, IndexError):
            continue
    return dict(rss_bytes=rss, live_cpu_ticks=ticks, threads=threads)


def stop_group(process):
    # Kill the entire owned process group, including compiler/test children.
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--lock', type=Path)
    parser.add_argument('--cpus', type=int, default=2)
    parser.add_argument('--min-available-gib', type=float, default=32)
    parser.add_argument('--max-rss-gib', type=float, default=8)
    parser.add_argument('--timeout', type=float, default=600)
    parser.add_argument('--gpu', type=int, help='require and monitor this physical GPU')
    parser.add_argument('--min-gpu-free-gib', type=float, default=8)
    parser.add_argument('--max-gpu-growth-gib', type=float, default=4)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command or args.cpus < 1 or args.timeout <= 0 or any(
            value < 0 for value in [args.min_available_gib, args.max_rss_gib,
                                   args.min_gpu_free_gib, args.max_gpu_growth_gib]):
        parser.error('provide a command and positive CPU/timeout, nonnegative memory limits')
    args.report.parent.mkdir(parents=True, exist_ok=True)
    lock_path = args.lock or args.report.parent / 'workstation.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    report = dict(command=command, limits=dict(cpus=args.cpus,
                  min_available_gib=args.min_available_gib, max_rss_gib=args.max_rss_gib,
                  timeout_seconds=args.timeout, gpu=args.gpu,
                  min_gpu_free_gib=args.min_gpu_free_gib,
                  max_gpu_growth_gib=args.max_gpu_growth_gib), samples=[], status='preflight')
    process = None
    started = time.monotonic()
    def interrupted(signum, _frame):
        raise RuntimeError('runner interrupted by signal ' + str(signum))
    previous_term = signal.signal(signal.SIGTERM, interrupted)
    try:
        with lock_path.open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
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
            previous_gpu = initial_gpu
            next_gpu_poll = started
            while True:
                now = time.monotonic()
                mem = memory_info()
                usage = group_usage(process.pid)
                if args.gpu is not None and now >= next_gpu_poll:
                    previous_gpu = gpu_info(args.gpu)
                    next_gpu_poll = now + 2
                sample = dict(elapsed_seconds=round(now - started, 3),
                              available_bytes=mem['MemAvailable'],
                              free_bytes=mem['MemFree'], **usage, gpu=previous_gpu)
                report['samples'].append(sample)
                reason = None
                if mem['MemAvailable'] < args.min_available_gib * GIB:
                    reason = 'available RAM fell below reserve'
                elif usage['rss_bytes'] > args.max_rss_gib * GIB:
                    reason = 'job RSS exceeded budget'
                elif previous_gpu and previous_gpu['free_bytes'] < args.min_gpu_free_gib * GIB:
                    reason = 'free GPU memory fell below reserve'
                elif previous_gpu and previous_gpu['used_bytes'] - initial_gpu['used_bytes'] > args.max_gpu_growth_gib * GIB:
                    reason = 'total GPU memory growth exceeded job allowance'
                elif now - started > args.timeout:
                    reason = 'command timeout'
                if reason:
                    raise RuntimeError(reason)
                if process.poll() is not None:
                    break
                time.sleep(0.25)
            # Do not leave subprocesses running after the command exits.
            stop_group(process)
            report['exit_code'] = process.returncode
            report['status'] = 'passed' if process.returncode == 0 else 'command_failed'
    except (OSError, RuntimeError, subprocess.SubprocessError, KeyboardInterrupt) as error:
        if process is not None:
            stop_group(process)
        report['status'] = 'blocked_or_stopped'
        report['reason'] = str(error)
        report['exit_code'] = 125
        print('bounded: stopped:', error, file=sys.stderr, flush=True)
    finally:
        signal.signal(signal.SIGTERM, previous_term)
        report['elapsed_seconds'] = round(time.monotonic() - started, 3)
        report['peak_sampled_rss_bytes'] = max((s['rss_bytes'] for s in report['samples']), default=0)
        args.report.write_text(json.dumps(report, indent=2) + '\n')
    return report.get('exit_code', 125)


if __name__ == '__main__':
    sys.exit(main())
