"""Tiny real PNG captures and media-tool stand-ins; no ffmpeg execution."""

import csv
import hashlib
import json
from pathlib import Path
import struct
import zlib


def png(value):
    def chunk(kind, data):
        return (struct.pack(">I", len(data)) + kind + data
                + struct.pack(">I", zlib.crc32(kind + data)))
    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", 2, 2, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress((b"\x00" + bytes([value]) * 6) * 2))
            + chunk(b"IEND", b""))


def write_index(directory, rows, metadata):
    with (directory / "frames.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    metadata["frame_index_sha256"] = hashlib.sha256(
        (directory / "frames.csv").read_bytes()).hexdigest()
    (directory / "manifest.json").write_text(json.dumps(metadata))


def capture(directory, recovered=False):
    directory.mkdir()
    rows = []
    for index, (epoch, time) in enumerate(((0, 0), (25, 0.000005), (27, 0.0000054))):
        image = png(index * 70)
        name = f"frame-{index:06d}.png"
        (directory / name).write_bytes(image)
        rows.append(dict(sample=index, epoch=epoch, accepted_time_s=time,
                         file=name, bytes=len(image), sha256=hashlib.sha256(image).hexdigest()))
    metadata = dict(schema="robo_dyna.physical_replay_capture.v1", complete_capture=True,
                    interpolated_frames=False, deformation_scale=1.0,
                    simulation_executed_by_viewer=False, all_png_decoded=True,
                    frames=len(rows), width=2, height=2,
                    png_bytes=sum(row["bytes"] for row in rows),
                    final_epoch=27, final_time_s=0.0000054,
                    input_horizon_complete=False, input_stop_reason="Diagnostic prefix",
                    recorded_samples_per_second=5.0,
                    input_receipt={"file": "viewer-input.json", "sha256": "a" * 64, "bytes": 8})
    if recovered:
        metadata.update(schema="robo_dyna.recovered_sample_capture.v1",
                        final_saved_epoch=metadata.pop("final_epoch"),
                        final_saved_time_s=metadata.pop("final_time_s"),
                        input_horizon_completion="unknown", interval_ledger_available=False,
                        continuous_accepted_history_available=False,
                        input_stop_reason="Interrupted; interval ledger unavailable",
                        input_recovery_descriptor={"file": "recovered-samples.json",
                                                   "sha256": "b" * 64, "bytes": 9})
        del metadata["input_horizon_complete"]
        del metadata["input_receipt"]
    write_index(directory, rows, metadata)
    return rows, metadata


def tools(directory):
    directory.mkdir()
    for name in ("ffmpeg", "ffprobe"):
        path = directory / name
        path.write_text("This fixture must never be executed.\n")
        path.chmod(0o700)
    return directory / "ffmpeg"
