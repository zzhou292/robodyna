"""Explicit renderer tools and resource policy, separate from simulation inputs."""

import os
from pathlib import Path
import re

from viewer.file_integrity import sha256_file
from ..jsonio import fields, integer, read_json, real, require, resolve
from ..manifests import GIB, MIB
from ..runtime import runtime_file
from .assets import load_assets

DISPLAY_KEYS = frozenset(("DISPLAY", "XAUTHORITY", "XDG_RUNTIME_DIR", "DBUS_SESSION_BUS_ADDRESS", "VK_ICD_FILENAMES"))
VIEWS = {"overview": ["--view", "incident-side"],
         "front": ["--camera-eye", "1.2,-1.6,1.0", "--camera-target", "-0.35,0,0.55", "--camera-up", "z"]}


def load_render_resources(path):
    path = Path(path).absolute()
    doc = read_json(path)
    fields(doc, ("schema", "cpu_threads", "rss_bytes", "minimum_available_ram_bytes", "gpu_index",
                 "minimum_gpu_free_bytes", "maximum_gpu_growth_bytes", "timeout_s", "workstation_lock",
                 "png_budget_bytes", "samples_per_second", "output_fps"))
    require(doc["schema"] == "robodyna.render_resources.v1", "unsupported render resource schema")
    result = dict(doc)
    for name in ("cpu_threads", "rss_bytes", "minimum_available_ram_bytes", "minimum_gpu_free_bytes",
                 "maximum_gpu_growth_bytes", "png_budget_bytes", "output_fps"):
        result[name] = integer(doc[name], name, 1)
    # The inherited viewer has no portable explicit Vulkan device selector.
    result["gpu_index"] = integer(doc["gpu_index"], "qualified render GPU index", 0, 0)
    result["timeout_s"] = real(doc["timeout_s"], "render timeout", positive=True)
    result["samples_per_second"] = real(doc["samples_per_second"], "sample playback rate", 1, 60)
    require(result["samples_per_second"] <= result["output_fps"], "every saved state needs an output video frame")
    require(result["png_budget_bytes"] in (2 * GIB, 6 * GIB, 10 * GIB), "PNG budget must be 2, 6 or 10 GiB")
    result["workstation_lock"] = str(resolve(path.parent, doc["workstation_lock"]))
    return result


def _tool(value, base):
    require(type(value) is dict and set(value) in ({"path", "sha256"}, {"runfile", "sha256"}),
            "tool requires one path/runfile and SHA-256")
    digest = value["sha256"]
    require(type(digest) is str and re.fullmatch(r"[0-9a-f]{64}", digest), "invalid tool SHA-256")
    if "runfile" in value:
        require(type(value["runfile"]) is str and 0 < len(value["runfile"]) <= 4096, "invalid tool runfile")
        path = runtime_file(value["runfile"])
    else:
        path = resolve(base, value["path"])
    path = path.resolve(strict=True)
    require(path.is_file() and os.access(path, os.X_OK) and sha256_file(path) == digest,
            f"pinned executable differs or is not executable: {path}")
    return dict(path=str(path), sha256=digest)


def load_tools(path):
    path = Path(path).absolute()
    doc = read_json(path)
    fields(doc, ("schema", "viewer", "ffmpeg", "ffprobe"), ("display_environment", "chrono_data"))
    require(doc["schema"] == "robodyna.render_tools.v1", "unsupported render tool schema")
    tools = {name: _tool(doc[name], path.parent) for name in ("viewer", "ffmpeg", "ffprobe")}
    require(Path(tools["ffmpeg"]["path"]).with_name("ffprobe").resolve() == Path(tools["ffprobe"]["path"]),
            "existing encoder requires the explicitly pinned sibling ffprobe")
    if "chrono_data" in doc:
        directory, assets = load_assets(doc["chrono_data"], path.parent)
        tools["viewer"]["chrono_data"] = directory
        tools.update({"asset:" + name: record for name, record in assets.items()})
    environment = doc.get("display_environment", {})
    require(type(environment) is dict and environment.keys() <= DISPLAY_KEYS, "unsupported display environment override")
    for key, value in environment.items():
        require(type(value) is str and 0 < len(value) <= 4096 and "\x00" not in value, f"invalid {key}")
    return tools, environment


def viewer_asset_arguments(viewer):
    return ["--chrono-data", viewer["chrono_data"]] if "chrono_data" in viewer else []


def capture_forecast(samples, resources, views):
    integer(samples, "saved states", 1, 10000)
    require(views and len(set(views)) == len(views) and all(view in VIEWS for view in views), "invalid or duplicate views")
    value = samples * 32 * MIB + 4 * MIB
    require(value <= resources["png_budget_bytes"], "saved-state PNG forecast exceeds the per-view disk allowance")
    return value
