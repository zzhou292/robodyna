"""Small hash-bound product receipts; not numerical qualification evidence."""

from pathlib import Path
from viewer.file_integrity import sha256_file
from .jsonio import fields, integer, read_json, require


def record(root, relative):
    root = Path(root).resolve()
    path = root / relative
    require(not path.is_symlink() and path.is_file() and path.resolve().is_relative_to(root),
            "receipt must bind a regular file inside its output root")
    return dict(file=relative, bytes=path.stat().st_size, sha256=sha256_file(path))


def verify(root, value, relative):
    fields(value, ("file", "bytes", "sha256"))
    require(value["file"] == relative, "receipt path differs from its declared role")
    integer(value["bytes"], "receipt bytes", 1)
    require(record(root, relative) == value, f"product receipt changed: {relative}")


def product_record_names(mode):
    require(mode in ("plan", "run"), "unknown product result mode")
    files = ("launch.json", "request.json", "native-report.json", "guard.json")
    if mode == "run":
        files += ("accepted/summary.json", "accepted/viewer-input.json", "accepted/archive/manifest.json")
    return files


def verify_product_records(root, result, mode):
    """Verify common product provenance; callers still own closure/physics checks."""
    require(type(result) is dict and result.get("schema") == "robodyna.launch_result.v1"
            and result.get("mode") == mode, "product result schema or mode differs")
    records = result.get("records")
    fields(records, product_record_names(mode))
    for name in product_record_names(mode):
        verify(root, records[name], name)
    launch = read_json(Path(root) / "launch.json")
    require(launch.get("schema") == "robodyna.launch.v1" and launch.get("mode") == mode
            and launch.get("request") == "request.json"
            and records["request.json"]["sha256"] == launch.get("request_sha256"),
            "product launch request identity differs")
    return records
