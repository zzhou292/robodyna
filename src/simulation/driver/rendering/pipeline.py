"""One render workflow for normal product runs and coherent legacy archives."""

from pathlib import Path
import sys

from viewer.file_integrity import sha256_file
from ..jsonio import read_json, require, write_new
from ..receipts import record
from ..runtime import runtime_file
from .config import VIEWS, capture_forecast, load_render_resources, load_tools, viewer_asset_arguments
from .inputs import render_input
from .stages import Stages


def render(path, resources_path, tools_path, destination, views=None, producer_guard=None, guard=None, watchdog_python=None):
    source = render_input(path, producer_guard)
    resources = load_render_resources(resources_path)
    tools, environment = load_tools(tools_path)
    views = list(views or ("overview", "front"))
    forecast = capture_forecast(source["saved_states"], resources, views)
    guard = runtime_file("legacy_fea/tools/run_bounded.py", guard)
    tools["watchdog"] = dict(path=str(guard), sha256=sha256_file(guard))
    output = Path(destination).absolute()
    require(not output.exists() and not output.is_symlink(), "render output must be a new directory")
    require(output.parent.is_dir(), "render output parent must exist")
    require(not output.resolve().is_relative_to(Path(source["output"]).resolve()),
            "render output must be outside the immutable accepted directory")
    output.mkdir()
    request = dict(schema="robodyna.render_request.v1", input=source, resources=resources, tools=tools,
                   views=views, output=str(output), capture_forecast_bytes_per_view=forecast,
                   display_environment=environment)
    request_path = output / "render-request.json"
    write_new(request_path, request)
    digest = sha256_file(request_path)
    stages = None
    worker = [sys.executable, "-B", "-m", "src.simulation.driver.rendering.worker"]
    def cpu_stage(operation, view=None):
        command = [*worker, operation, str(request_path), "--request-sha256", digest]
        if view is not None:
            command += ["--view", view]
        stages.run(operation if view is None else view + "-encode", command)
    try:
        stages = Stages(output, resources, tools, environment, guard, watchdog_python)
        cpu_stage("inventory-before")
        videos = []
        for view in views:
            capture = output / (view + "-capture")
            stages.run(view + "-capture", [tools["viewer"]["path"], source["output"], "--capture", str(capture),
                "--capture-cap-gib", str(resources["png_budget_bytes"] >> 30), "--color", "part-id",
                "--part-palette-seed", "2", "--fps", str(resources["samples_per_second"]),
                "--require-frames", str(source["saved_states"]), "--receipt-sha256", source["viewer_input_sha256"],
                *viewer_asset_arguments(tools["viewer"]), *VIEWS[view]], gpu=True)
            cpu_stage("encode", view)
            directory = output / (view + "-video")
            manifest = read_json(directory / "manifest.json", 2 << 20)
            require(manifest.get("full_decode_passed") is True, "video lacks full decode verification")
            videos.append(dict(view=view, path=str(directory / "movie.mp4"), video=manifest["video"],
                               manifest=record(output, view + "-video/manifest.json")))
        cpu_stage("inventory-after")
        require(render_input(path, producer_guard) == source, "input closure metadata changed during rendering")
        result = dict(schema="robodyna.render_result.v1", status="media_verified", input=source, videos=videos,
                      request=record(output, "render-request.json"), stage_guards=stages.receipts,
                      inventory_before=record(output, "inventory-before.json"), inventory_after=record(output, "inventory-after.json"),
                      accepted_inputs_unchanged=True, visual_review="pending", numerical_qualification="not_inferred_from_rendering")
        write_new(output / "render-result.json", result)
        return result
    except Exception as error:
        write_new(output / "render-failure.json", dict(schema="robodyna.render_failure.v1", error=str(error),
                  stage_guards=stages.receipts if stages is not None else {}))
        raise
