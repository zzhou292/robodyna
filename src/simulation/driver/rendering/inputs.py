"""Keep coherent media input distinct from numerical qualification."""

from pathlib import Path

from ..inspection import inspect_run
from ..jsonio import read_json, require
from ..job import verify_completion
from ..receipts import verify_product_records


def render_input(path, producer_guard=None):
    path = Path(path).absolute()
    if (path / "launch.json").exists():
        require(producer_guard is None, "product launch cannot be downgraded to a legacy receipt path")
        launch = read_json(path / "launch.json")
        require(launch.get("schema") == "robodyna.launch.v1" and launch.get("mode") == "run"
                and launch.get("output") == "accepted" and launch.get("guard_report") == "guard.json"
                and launch.get("request") == "request.json", "invalid product run launch")
        result = read_json(path / "launch-result.json")
        require(result.get("schema") == "robodyna.launch_result.v1" and result.get("mode") == "run"
                and result.get("product_result_verified") is True and type(result.get("return_code")) is int
                and result["return_code"] in (0, 2), "product run did not publish a verified closed result")
        verify_product_records(path, result, "run")
        native = read_json(path / "native-report.json")
        verify_completion(path, "run", result["return_code"], native)
        inspection = inspect_run(path)
        source_kind = "normal_product_run"
    else:
        require(producer_guard is not None, "legacy accepted archive requires its explicit producer guard")
        inspection = inspect_run(path, producer_guard)
        source_kind = "legacy_closed_archive"
    accepted = Path(inspection["output"])
    summary = read_json(accepted / "summary.json")
    return dict(**inspection, input_kind=source_kind,
                archive_manifest_sha256=summary["archive_manifest"]["sha256"],
                numerical_qualification="not_inferred_from_rendering")
