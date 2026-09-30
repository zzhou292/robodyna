"""Bounded compute-process telemetry query; never changes GPU state."""

from dataclasses import dataclass
from typing import Optional
import os
import selectors
import subprocess
import time

OUTPUT_CAP = 64 * 1024
QUERY_TIMEOUT_SECONDS = 1.0


@dataclass(frozen=True)
class QueryResult:
    status: str
    data: bytes = b''
    returncode: Optional[int] = None


def query_compute_apps(index):
    """Drain at most OUTPUT_CAP bytes and always reap this direct query child.

    stderr is deliberately discarded. A failed/truncated query has no complete
    process inventory. The bounded timeout includes pipe drain and child wait.
    """
    process = None
    try:
        deadline = time.monotonic() + QUERY_TIMEOUT_SECONDS
        process = subprocess.Popen(
            ['nvidia-smi', '-i', str(index),
             '--query-compute-apps=pid,used_gpu_memory',
             '--format=csv,noheader,nounits'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        output = bytearray()
        with selectors.DefaultSelector() as selector:
            selector.register(process.stdout, selectors.EVENT_READ)
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return QueryResult('query_timeout')
                if not selector.select(remaining):
                    return QueryResult('query_timeout')
                chunk = os.read(process.stdout.fileno(),
                                min(4096, OUTPUT_CAP + 1 - len(output)))
                if not chunk:
                    break
                output.extend(chunk)
                if len(output) > OUTPUT_CAP:
                    return QueryResult('query_output_limit')
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return QueryResult('query_timeout')
            code = process.wait(timeout=remaining)
            if code:
                return QueryResult('query_failed', returncode=code)
            return QueryResult('ok', bytes(output), code)
    except subprocess.TimeoutExpired:
        return QueryResult('query_timeout')
    except (OSError, ValueError, subprocess.SubprocessError):
        return QueryResult('query_unavailable')
    finally:
        if process is not None:
            try:
                if process.poll() is None:
                    process.kill()
                process.wait(timeout=1)
            finally:
                if process.stdout is not None:
                    process.stdout.close()
