"""Render contracts with tiny files and mocked processes; never invoke media tools."""

from pathlib import Path
import tempfile
import unittest
import sys
from unittest.mock import patch

from src.simulation.driver.jsonio import read_json, write_new
from src.simulation.driver.manifests import GIB, MIB
from src.simulation.driver.receipts import product_record_names, record, verify, verify_product_records
from src.simulation.driver.rendering.config import capture_forecast, load_render_resources, load_tools
from src.simulation.driver.rendering.inputs import render_input
from src.simulation.driver.rendering.integrity import inventory
from src.simulation.driver.rendering.stages import Stages
from src.simulation.driver.rendering.pipeline import render
from src.simulation.driver.rendering.worker import capture_identity
from .fixture import digest, write


class RenderTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def resource_document(self):
        return dict(schema="robodyna.render_resources.v1", cpu_threads=2, rss_bytes=10 * GIB,
                    minimum_available_ram_bytes=32 * GIB, gpu_index=0, minimum_gpu_free_bytes=8 * GIB,
                    maximum_gpu_growth_bytes=8 * GIB, timeout_s=1200, workstation_lock="workstation.lock",
                    png_budget_bytes=10 * GIB, samples_per_second=5, output_fps=30)

    def tools(self):
        tools = dict(schema="robodyna.render_tools.v1")
        for name in ("viewer", "ffmpeg", "ffprobe"):
            path = self.root / name
            data = (name + " fixture - never executed").encode()
            path.write_bytes(data)
            path.chmod(0o700)
            tools[name] = dict(path=path.name, sha256=digest(data))
        return tools

    def test_render_resources_and_png_capacity_are_distinct_from_solver_archive(self):
        write(self.root / "resources.json", self.resource_document())
        resources = load_render_resources(self.root / "resources.json")
        self.assertEqual(capture_forecast(301, resources, ["overview", "front"]), 301 * 32 * MIB + 4 * MIB)
        resources["png_budget_bytes"] = 6 * GIB
        with self.assertRaisesRegex(ValueError, "per-view"):
            capture_forecast(301, resources, ["front"])
        with self.assertRaises(ValueError):
            capture_forecast(10, resources, ["front", "front"])

    def test_tool_pin_change_rejects_before_any_media_process(self):
        tools = self.tools()
        write(self.root / "tools.json", tools)
        selected, environment = load_tools(self.root / "tools.json")
        self.assertEqual(environment, {})
        self.assertEqual(selected["ffprobe"]["path"], str(self.root / "ffprobe"))
        (self.root / "viewer").write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "pinned executable"):
            load_tools(self.root / "tools.json")

    def test_product_receipts_detect_file_change_and_path_substitution(self):
        (self.root / "report.json").write_text("original")
        value = record(self.root, "report.json")
        verify(self.root, value, "report.json")
        with self.assertRaises(ValueError):
            verify(self.root, {**value, "file": "../report.json"}, "report.json")
        (self.root / "report.json").write_text("changed")
        with self.assertRaisesRegex(ValueError, "changed"):
            verify(self.root, value, "report.json")

    def test_shared_product_record_gate_rejects_tampering_and_wrong_request_link(self):
        write_new(self.root / "request.json", dict(request="metadata-only fixture"))
        request_hash = record(self.root, "request.json")["sha256"]
        write(self.root / "launch.json", dict(schema="robodyna.launch.v1", mode="plan",
              request="request.json", request_sha256=request_hash))
        for name in ("guard.json", "native-report.json"):
            write_new(self.root / name, dict(fixture="not a physical run"))
        result = dict(schema="robodyna.launch_result.v1", mode="plan",
                      records={name: record(self.root, name) for name in product_record_names("plan")})
        verify_product_records(self.root, result, "plan")
        changed = {**result, "records": {**result["records"],
                   "native-report.json": {**result["records"]["native-report.json"], "sha256": "0" * 64}}}
        with self.assertRaisesRegex(ValueError, "changed"):
            verify_product_records(self.root, changed, "plan")
        write(self.root / "launch.json", dict(schema="robodyna.launch.v1", mode="plan",
              request="request.json", request_sha256="0" * 64))
        result["records"]["launch.json"] = record(self.root, "launch.json")
        with self.assertRaisesRegex(ValueError, "request identity"):
            verify_product_records(self.root, result, "plan")

    def test_missing_product_result_cannot_fall_back_to_legacy_rendering(self):
        write_new(self.root / "launch.json", dict(schema="robodyna.launch.v1", mode="run",
                  output="accepted", guard_report="guard.json", request="request.json"))
        with self.assertRaisesRegex(ValueError, "downgraded"):
            render_input(self.root, self.root / "guard.json")
        with self.assertRaisesRegex(ValueError, "launch-result"):
            render_input(self.root)

    def test_legacy_render_input_reuses_closed_metadata_without_requiring_tests(self):
        accepted = self.root / "accepted"
        accepted.mkdir()
        write_new(accepted / "summary.json", dict(archive_manifest=dict(sha256="a" * 64)))
        inspected = dict(output=str(accepted), horizon_complete=True, accepted_intervals=2,
                         actual_time_s=3e-7, fixed_dt_s=1.5e-7, saved_states=3, viewer_input_sha256="b" * 64)
        with patch("src.simulation.driver.rendering.inputs.inspect_run", return_value=inspected):
            result = render_input(accepted, self.root / "producer-guard.json")
        self.assertEqual(result["input_kind"], "legacy_closed_archive")
        self.assertEqual(result["numerical_qualification"], "not_inferred_from_rendering")

    def test_capture_from_another_archive_or_wrong_physical_scale_rejects(self):
        expected = dict(viewer_input_sha256="a" * 64, archive_manifest_sha256="b" * 64,
                        saved_states=3, accepted_intervals=2, actual_time_s=3e-7, horizon_complete=True)
        metadata = dict(schema="robo_dyna.physical_replay_capture.v1", complete_capture=True,
                        input_receipt=dict(sha256="a" * 64), input_archive_manifest=dict(sha256="b" * 64),
                        frames=3, final_epoch=2, final_time_s=3e-7, input_horizon_complete=True,
                        surface_color_mode="part-id", part_palette_seed=2, deformation_scale=1,
                        interpolated_frames=False, simulation_executed_by_viewer=False)
        capture_identity(metadata, expected)
        for change in (dict(final_epoch=3), dict(deformation_scale=2), dict(interpolated_frames=True)):
            with self.assertRaises(ValueError):
                capture_identity({**metadata, **change}, expected)

    def test_inventory_detects_content_change_and_refuses_symbolic_links(self):
        source = self.root / "accepted"
        source.mkdir()
        (source / "file").write_text("original")
        first = inventory(source)
        (source / "file").write_text("changed")
        self.assertNotEqual(first, inventory(source))
        (source / "linked").symlink_to(source / "file")
        with self.assertRaisesRegex(ValueError, "symbolic link"):
            inventory(source)

    def test_zero_stage_exit_without_cleanup_receipt_does_not_authorize_next_stage(self):
        tools = self.tools()
        write(self.root / "tools.json", tools)
        selected, display = load_tools(self.root / "tools.json")
        resources = self.resource_document()
        resources["workstation_lock"] = str(self.root / "lock")
        (self.root / "guard.py").write_text("# mock-only stage tool; never executed\n")
        with patch("src.simulation.driver.rendering.stages.select_watchdog_interpreter",
                   return_value=dict(path=sys.executable, capability="mock-only-no-process")):
            stages = Stages(self.root, resources, selected, display, self.root / "guard.py")
        with patch("src.simulation.driver.rendering.stages.subprocess.run") as run:
            run.return_value.returncode = 0
            with self.assertRaisesRegex(ValueError, "capture.guard"):
                stages.run("capture", [str(self.root / "viewer")], gpu=True)
        self.assertEqual(stages.receipts, {})

    def test_watchdog_selection_failure_preserves_request_and_publishes_failure_without_process(self):
        accepted = self.root / "accepted"
        accepted.mkdir()
        guard = self.root / "guard.py"
        guard.write_text("# identity fixture; never executed\n")
        output = self.root / "render"
        source = dict(output=str(accepted), saved_states=3)
        with patch("src.simulation.driver.rendering.pipeline.render_input", return_value=source), \
             patch("src.simulation.driver.rendering.pipeline.load_render_resources", return_value=self.resource_document()), \
             patch("src.simulation.driver.rendering.pipeline.load_tools", return_value=({}, {})), \
             patch("src.simulation.driver.rendering.pipeline.runtime_file", return_value=guard), \
             patch("src.simulation.driver.rendering.stages.select_watchdog_interpreter",
                   side_effect=ValueError("unsupported watchdog interpreter")), \
             patch("src.simulation.driver.rendering.stages.subprocess.run") as process:
            with self.assertRaisesRegex(ValueError, "unsupported watchdog"):
                render(accepted, self.root / "resources.json", self.root / "tools.json", output)
        process.assert_not_called()
        self.assertTrue((output / "render-request.json").is_file())
        failure = read_json(output / "render-failure.json")
        self.assertEqual(failure["error"], "unsupported watchdog interpreter")
        self.assertEqual(failure["stage_guards"], {})
        self.assertFalse((output / "render-result.json").exists())
        self.assertEqual(list(output.glob("*.guard.json")), [])


if __name__ == "__main__":
    unittest.main()
