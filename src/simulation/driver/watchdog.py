"""Select a Python runtime that supports the unchanged Linux process monitor."""

import json
import os
from pathlib import Path
import subprocess
import sys

from viewer.file_integrity import sha256_file
from .jsonio import require
from .runtime import watchdog_environment

PROBE = (
    "import json,sys; from bounded_session import preflight; preflight(); "
    "print(json.dumps({'executable':sys.executable,'version':sys.version.split()[0],"
    "'capability':'owned_session_v1'}))"
)


def select_watchdog_interpreter(guard, environment, explicit=None, *, current_interpreter=None):
    """Probe real pidfd/waitid support; never substitute a weaker monitor."""
    candidates = [("explicit", explicit)] if explicit else [
        ("cli_python", current_interpreter or sys.executable), ("platform_python", "/usr/bin/python3")]
    environment = watchdog_environment(environment, guard)
    rejected, seen = [], set()
    for origin, value in candidates:
        path = Path(value).absolute().resolve()
        if path in seen:
            continue
        seen.add(path)
        if not path.is_file() or not os.access(path, os.X_OK):
            rejected.append(dict(path=str(path), reason="interpreter is not executable"))
            continue
        try:
            result = subprocess.run([str(path), "-B", "-c", PROBE], env=environment,
                                    capture_output=True, text=True, timeout=10, check=False)
            if result.returncode != 0:
                detail = result.stderr.strip().splitlines()
                rejected.append(dict(path=str(path), reason=detail[-1][:512] if detail else "capability probe failed"))
                continue
            info = json.loads(result.stdout)
            require(info.get("capability") == "owned_session_v1", "unexpected watchdog capability response")
            return dict(path=str(path), version=info["version"], sha256=sha256_file(path), selection=origin,
                        capability=info["capability"], rejected_candidates=rejected)
        except (OSError, ValueError, KeyError, subprocess.TimeoutExpired) as error:
            rejected.append(dict(path=str(path), reason=str(error)[:512]))
    reason = "; ".join(value["path"] + ": " + value["reason"] for value in rejected)
    raise ValueError("No watchdog Python supports the required pidfd/waitid monitor. " + reason)
