"""Serialize render work through the existing workstation resource guard."""

import os
from pathlib import Path
import subprocess
import viewer.file_integrity

from ..jsonio import read_json, require, write_new
from ..receipts import record
from ..runtime import clean_environment, monitored_command, watchdog_environment
from ..watchdog import select_watchdog_interpreter
from .worker import check_tools


class Stages:
    def __init__(self, output, resources, tools, display, guard, watchdog_python=None):
        self.output, self.resources, self.tools, self.guard = output, resources, tools, guard
        self.environment = clean_environment(resources["gpu_index"])
        self.environment.update(display)
        # Workers need these two declared package roots, not the CLI runtime's
        # standard-library directories, which could poison a platform Python.
        roots = (Path(__file__).resolve().parents[4], Path(viewer.file_integrity.__file__).resolve().parents[1])
        self.environment["PYTHONPATH"] = os.pathsep.join(map(str, roots))
        self.environment = watchdog_environment(self.environment, guard)
        self.interpreter = select_watchdog_interpreter(guard, self.environment, watchdog_python)
        self.receipts = {}

    def run(self, name, command, gpu=False):
        check_tools(self.tools)
        report = self.output / (name + ".guard.json")
        argv = monitored_command(self.guard, self.resources, report, command, self.interpreter["path"], gpu=gpu)
        write_new(self.output / (name + ".command.json"), dict(command=argv, gpu_guard=gpu, watchdog_interpreter=self.interpreter))
        print(f"render: {name}; log: {self.output / (name + '.log')}", flush=True)
        with (self.output / (name + ".log")).open("xb") as log:
            completed = subprocess.run(argv, env=self.environment, stdout=log, stderr=subprocess.STDOUT, check=False)
        require(completed.returncode == 0, f"{name} failed its bounded process (exit {completed.returncode})")
        result = read_json(report, 16 << 20)
        require(type(result.get("exit_code")) is int and result["exit_code"] == 0 and result.get("status") == "passed"
                and result.get("process_scope", {}).get("cleanup") == "complete", f"{name} lacks successful guard cleanup")
        check_tools(self.tools)
        self.receipts[name] = record(self.output, report.name)
