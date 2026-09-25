from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from benchmarks.case_parity.performance import assess
from .support import Fixture


class PerformanceTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory(); self.addCleanup(tmp.cleanup)
        self.fixture = Fixture(tmp.name)

    def test_synthetic_matching_evidence_yields_only_a_scoped_measured_win(self):
        f = self.fixture; result = assess(f.request(), f.root)
        self.assertEqual(result["status"], "measured_gpu_win")
        self.assertEqual(result["case_scope"], "synthetic")
        self.assertFalse(result["full_vehicle_requirement_met"])
        self.assertEqual(result["gpu_speedup"]["end_to_end"]["range_separation_lower_bound"], 3)

    def test_completed_named_test_with_producer_and_contract_bindings_is_required(self):
        for mutation in ("wrong_name", "skipped", "missing_properties", "wrong_producer"):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as directory:
                f = Fixture(directory); evidence = f.load(f.numerics)
                tree = ET.fromstring(Path(evidence["xml"]["path"]).read_bytes())
                case = tree[0][0]
                if mutation == "wrong_name": case.set("name", "Other")
                if mutation == "skipped": case.set("result", "skipped")
                if mutation == "missing_properties": case.remove(case[0])
                if mutation == "wrong_producer": case[0][2].set("value", "0" * 64)
                evidence["xml"] = f.write("numerics.xml", ET.tostring(tree))
                f.numerics = f.write("numerics.json", evidence)
                with self.assertRaises(ValueError): assess(f.request(refresh_numerics=False), f.root)

    def test_guard_failure_incomplete_prefix_and_solver_only_boundary_reject(self):
        mutations = (
            lambda d: d.update(completed_steps=7),
            lambda d: d.update(start_time_s=0, end_time_s=0),
            lambda d: d.update(timing_scope="solver_only"),
            lambda d: d.update(producer_sha256="0"*64),
        )
        for mutate in mutations:
            with self.subTest(mutate=mutate), tempfile.TemporaryDirectory() as directory:
                f = Fixture(directory)
                f.pairs[0]["candidate"] = f.edit_run(f.pairs[0]["candidate"], mutate)
                with self.assertRaises(ValueError): assess(f.request(), f.root)
        f = self.fixture; row = f.load(f.pairs[0]["candidate"])
        row["guard"] = f.change(row["guard"], lambda d: d.update(exit_code=2, status="command_failed"))
        f.pairs[0]["candidate"] = f.write("pair0-gpu.json", row)
        with self.assertRaises(ValueError): assess(f.request(), f.root)

    def test_time_output_resource_and_platform_work_must_match(self):
        for key, value in (("end_time_s", 9e-6), ("platform_id", "different-host")):
            with self.subTest(key=key), tempfile.TemporaryDirectory() as directory:
                f = Fixture(directory)
                f.pairs[0]["candidate"] = f.edit_run(f.pairs[0]["candidate"], lambda d: d.update({key: value}))
                with self.assertRaises(ValueError): assess(f.request(), f.root)
        f = self.fixture
        other = f.write("different-output.json", {"sample_epochs": [0, 4, 8]})
        f.pairs[0]["candidate"] = f.edit_run(f.pairs[0]["candidate"], lambda d: d.update(output_work=other))
        with self.assertRaises(ValueError): assess(f.request(), f.root)

    def test_duplicates_and_false_interleaving_cannot_inflate_repetitions(self):
        f = self.fixture
        duplicate = [f.pairs[0], f.pairs[1], f.pairs[0]]
        with self.assertRaises(ValueError): assess(f.request(pairs=duplicate), f.root)
        f.pairs[1]["order"] = "reference_candidate"
        with self.assertRaises(ValueError): assess(f.request(), f.root)

    def test_two_pairs_and_a_warm_only_regression_do_not_admit_a_win(self):
        f = self.fixture
        result = assess(f.request(pairs=f.pairs[:2]), f.root)
        self.assertEqual(result["status"], "comparable_without_speed_win")
        for pair in f.pairs:
            # Candidate total1s remains within its guard; reference warm .4s wins instead.
            pair["reference"] = f.edit_warm(pair["reference"], lambda d: d.update(total_seconds=.4))
        result = assess(f.request(), f.root)
        self.assertEqual(result["status"], "comparable_without_speed_win")

    def test_guard_cannot_name_a_producer_as_an_argument_to_true_or_forecast(self):
        for mode in ("wrong_executable", "forecast"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                f = Fixture(directory); pair = f.pairs[0]; wrapper = f.load(pair["candidate"])
                def mutate(g):
                    if mode == "wrong_executable": g["command"].insert(0, "/bin/true")
                    else: g["command"].append("--forecast-only")
                wrapper["guard"] = f.change(wrapper["guard"], mutate)
                pair["candidate"] = f.write("pair0-gpu.json", wrapper)
                with self.assertRaises(ValueError): assess(f.request(), f.root)

    def test_warm_windows_and_actual_declared_time_coverage_are_bound(self):
        for field, value in (("first_step", 4), ("step_count", 5), ("boundary", "contact_only")):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as directory:
                f = Fixture(directory)
                f.pairs[0]["candidate"] = f.edit_warm(f.pairs[0]["candidate"],
                    lambda d: d.update({field: value}))
                with self.assertRaises(ValueError): assess(f.request(), f.root)
        f = self.fixture
        # Both sides claim2 steps, but their contract and time grid still demand8.
        for pair in f.pairs:
            for role in ("reference", "candidate"):
                pair[role] = f.edit_run(pair[role], lambda d: d.update(requested_steps=2, completed_steps=2))
        with self.assertRaises(ValueError): assess(f.request(), f.root)

    def test_time_grid_and_output_payload_extents_are_parsed_not_only_hashed(self):
        for kind in ("grid", "payload"):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as directory:
                f = Fixture(directory)
                if kind == "grid":
                    pin = f.change(f.grid, lambda d: d.update(times_s=[0, 8e-6]))
                    field = "time_grid"
                else:
                    work = f.load(f.output)
                    work["payloads"]["position_m"] = f.write("bad-payload.bin", b"x")
                    pin = f.write("bad-output.json", work); field = "output_work"
                f.pairs[0]["candidate"] = f.edit_run(f.pairs[0]["candidate"],
                    lambda d: d.update({field: pin}))
                with self.assertRaises(ValueError): assess(f.request(), f.root)

    def test_preflight_duration_is_not_added_to_child_start_ticks(self):
        f = self.fixture
        for pair in f.pairs:
            for role in ("reference", "candidate"):
                pin = pair[role]; wrapper = f.load(pin)
                # A late child start can be beyond the subsequent child's raw
                # start plus elapsed under the old mixed-origin calculation.
                wrapper["guard"] = f.change(wrapper["guard"],
                    lambda d: d["process_scope"].update(leader_start_ticks=999999999))
                wrapper["launcher_interval"] = f.change(wrapper["launcher_interval"],
                    lambda d: d.update(guard_sha256=wrapper["guard"]["sha256"]))
                pair[role] = f.write(Path(pin["path"]).name, wrapper)
        self.assertEqual(assess(f.request(), f.root)["status"], "measured_gpu_win")

    def test_late_producer_mutation_is_not_hidden_by_cached_pins(self):
        f = self.fixture
        Path(f.gpu["path"]).write_bytes(b"replaced producer")
        with self.assertRaises(ValueError): assess(f.request(), f.root)


if __name__ == "__main__":
    unittest.main()
