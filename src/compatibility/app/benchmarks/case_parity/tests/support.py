"""Synthetic records exercise consistency checks; none is real performance evidence."""
from pathlib import Path
import hashlib
import json
import struct
import xml.etree.ElementTree as ET
from benchmarks.case_parity.contracts import DOMAINS


class Fixture:
    def __init__(self, directory, scope="synthetic", steps=8):
        self.root = Path(directory); self.tick = 10000; self.launch_ns = 1000000000
        self.scope = scope; self.steps = steps
        self.declaration = self.write("declaration.json", {"scope": "synthetic parser fixture only"})
        self.tolerance = self.write("tolerance.json", {"scope": "synthetic", "absolute_force_n": 1e-12})
        self.cpu = self.write("cpu.producer", b"synthetic CPU producer identity; never executed")
        self.gpu = self.write("gpu.producer", b"synthetic GPU producer identity; never executed")
        self.comparator = self.write("comparator.producer", b"synthetic comparator identity; never executed")
        self.grid = self.write("grid.json", {"schema": "robo_dyna.case_time_grid.v1",
            "kind": "physical_steps", "times_s": [i*1e-6 for i in range(steps + 1)]})
        self.output_definition = {"precision": "binary64", "format": "case_fields_f64_v1",
            "fields": [{"name": "position_m", "components": 3}, {"name": "force_n", "components": 3}],
            "sample_epochs": [0, steps]}
        self.ref = self.contract("reference"); self.can = self.contract("candidate")
        self.pairs = [self.pair(i) for i in range(3)]
        self.numerics = self.numerical(self.pairs)

    def write(self, name, value):
        path = self.root / name
        path.write_bytes(value if isinstance(value, bytes) else json.dumps(value, allow_nan=False).encode())
        return self.pin(path)

    @staticmethod
    def pin(path):
        return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}

    @staticmethod
    def load(pin):
        return json.loads(Path(pin["path"]).read_text())

    def change(self, pin, operation):
        value = self.load(pin); operation(value)
        return self.write(Path(pin["path"]).name, value)

    def edit_run(self, pin, operation):
        wrapper = self.load(pin)
        wrapper["producer_record"] = self.change(wrapper["producer_record"], operation)
        wrapper["launcher_interval"] = self.change(wrapper["launcher_interval"],
            lambda d: d.update(producer_record_sha256=wrapper["producer_record"]["sha256"]))
        return self.write(Path(pin["path"]).name, wrapper)

    def edit_warm(self, pin, operation):
        def update(record):
            record["warm_timing"] = self.change(record["warm_timing"], operation)
        return self.edit_run(pin, update)

    def contract(self, name):
        domains = {domain: {"status": "resolved", "definition": {"synthetic_domain": domain},
                            "evidence": [self.declaration]} for domain in DOMAINS}
        domains["recorded_time_grid"]["definition"] = {"kind": "physical_steps", "planned_steps": self.steps,
            "initial_time_s": 0.0, "requested_end_time_s": self.steps * 1e-6}
        domains["output_work_precision"]["definition"] = self.output_definition
        return self.write(name + ".json", {
            "schema": "robo_dyna.resolved_case_contract.v1", "case_id": name, "scope": self.scope,
            "domains": domains, "numerical_protocol": {
                "id": "synthetic_evidence_only", "test_name": "ParityFixture.CompletedComparison",
                "tolerances": self.tolerance}})

    def guard(self, name, command, elapsed, cpus=4):
        self.tick += 1000
        start = self.tick; self.tick += int(elapsed * 100) + 100
        return self.write(name, {"status": "passed", "exit_code": 0, "command": command,
            "elapsed_seconds": elapsed, "limits": {"cpus": cpus}, "cpu_affinity": list(range(cpus)),
            "process_scope": {"cleanup": "complete", "leader_start_ticks": start}})

    def numerical(self, pairs):
        manifest = self.write("timing-manifest.json", {"schema": "robo_dyna.case_timing_manifest.v1", "pairs": pairs})
        report = ET.Element("testsuites", tests="1", failures="0", errors="0", disabled="0")
        suite = ET.SubElement(report, "testsuite", name="ParityFixture", tests="1",
                              failures="0", errors="0", disabled="0", skipped="0")
        case = ET.SubElement(suite, "testcase", classname="ParityFixture",
            name="CompletedComparison", status="run", result="completed")
        props = ET.SubElement(case, "properties")
        for key, value in {
            "reference_contract_sha256": self.ref["sha256"],
            "candidate_contract_sha256": self.can["sha256"],
            "reference_producer_sha256": self.cpu["sha256"],
            "candidate_producer_sha256": self.gpu["sha256"],
            "numerical_protocol": "synthetic_evidence_only",
            "tolerances_sha256": self.tolerance["sha256"],
            "timing_manifest_sha256": manifest["sha256"],
        }.items():
            ET.SubElement(props, "property", name=key, value=value)
        xml = self.write("numerics.xml", ET.tostring(report))
        guard = self.guard("numerics.guard.json", [self.comparator["path"],
            "--gtest_filter=ParityFixture.CompletedComparison",
            "--gtest_output=xml:" + xml["path"]], 1)
        return self.write("numerics.json", {
            "schema": "robo_dyna.case_numerical_evidence.v1",
            "reference_contract_sha256": self.ref["sha256"],
            "candidate_contract_sha256": self.can["sha256"],
            "reference_producer": self.cpu, "candidate_producer": self.gpu,
            "comparator": self.comparator, "xml": xml, "guard": guard, "timing_manifest": manifest})

    def run(self, pair, candidate, elapsed):
        name = f"pair{pair}-" + ("gpu" if candidate else "cpu")
        producer, contract = (self.gpu, self.can) if candidate else (self.cpu, self.ref)
        payloads = {field: self.write(name+"."+field+".bin", struct.pack("<6d", *range(6)))
                    for field in ("position_m", "force_n")}
        output = self.write(name+".output.json", {"schema": "robo_dyna.case_output_work.v1",
            "run_id": name, "producer_sha256": producer["sha256"],
            "contract_sha256": contract["sha256"],
            "definition": self.output_definition, "payloads": payloads})
        warmup = min(2, self.steps - 1)
        warm = self.write(name+".timing.json", {
            "schema": "robo_dyna.complete_advancement_timing.v1", "run_id": name,
            "producer_sha256": producer["sha256"], "contract_sha256": contract["sha256"],
            "boundary": "accepted_complete_step", "first_step": warmup + 1,
            "step_count": self.steps - warmup, "warmup_steps": warmup,
            "total_seconds": .5 if candidate else 2, "timer_resolution_seconds": 1e-9})
        record = self.write(name+".record.json", {
            "schema": "robo_dyna.completed_case_measurement.v1", "run_id": name,
            "backend": "cuda_candidate" if candidate else "cpu_reference",
            "contract_sha256": contract["sha256"], "producer_sha256": producer["sha256"],
            "requested_steps": self.steps, "completed_steps": self.steps,
            "start_time_s": 0, "end_time_s": self.steps * 1e-6,
            "time_grid": self.grid, "output_work": output,
            "timing_scope": "startup_advance_and_equivalent_output", "warm_timing": warm,
            "platform_id": "synthetic-host-and-boot"})
        guard = self.guard(name+".guard.json", [producer["path"], "--run-id", name,
                           "--benchmark-record", record["path"]], elapsed)
        started = self.launch_ns
        self.launch_ns += int(elapsed * 1e9)
        interval = self.write(name+".launcher.json", {
            "schema": "robo_dyna.benchmark_launcher_interval.v1", "clock": "CLOCK_MONOTONIC_NS",
            "platform_id": "synthetic-host-and-boot", "guard_sha256": guard["sha256"],
            "producer_record_sha256": record["sha256"], "start_ns": started, "end_ns": self.launch_ns})
        self.launch_ns += 1000000
        return self.write(name+".json", {"schema": "robo_dyna.guarded_case_measurement.v1",
                          "guard": guard, "producer_record": record, "launcher_interval": interval})

    def pair(self, index):
        if index % 2:
            b = self.run(index, True, 1); a = self.run(index, False, 3)
        else:
            a = self.run(index, False, 3); b = self.run(index, True, 1)
        return {"order": "candidate_reference" if index % 2 else "reference_candidate",
                "reference": a, "candidate": b}

    def request(self, numerics=True, pairs=None, refresh_numerics=True):
        pairs = self.pairs if pairs is None else pairs
        if numerics and refresh_numerics:
            self.numerics = self.numerical(pairs)
        return self.write("request.json", {"schema": "robo_dyna.case_parity_request.v1",
            "reference": self.ref, "candidate": self.can,
            "numerics": self.numerics if numerics else None, "timing_pairs": pairs})
