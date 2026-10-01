"""Overlay-free delivery admission and failure atomicity; no media execution."""

import copy
from dataclasses import asdict
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from viewer.file_integrity import sha256_file
from viewer.video.capture import validate_capture
from viewer.video.encode import hold_boundaries
from viewer.video.publication import export_video, main
from viewer.video.tests.fixtures import capture, png, tools


class PublicationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.capture = self.root / "capture"
        _, self.metadata = capture(self.capture)
        captured = validate_capture(self.capture)
        self.source = self.root / "native"
        self.source.mkdir()
        self.movie = self.source / "movie.mp4"
        self.movie.write_bytes(b"original authenticated native movie")
        boundaries = hold_boundaries(len(captured.frames), 5, 30)
        self.manifest = {
            "schema": "robo_dyna.chrono_capture_video.v1", "capture": str(self.capture),
            "full_decode_passed": True, "interpolated_frames": False, "deformation_scale": 1,
            "capture_metadata": self.metadata, "capture_manifest_sha256": captured.manifest_sha256,
            "frame_index_sha256": captured.frame_index_sha256, "samples_per_second": 5,
            "video": {"file": "movie.mp4", "frames": 18, "fps": 30, "duration_s": .6,
                      "bytes": self.movie.stat().st_size, "sha256": sha256_file(self.movie)},
            "recorded_samples": [{**asdict(frame), "first_video_frame": begin,
                                  "video_frames": end - begin, "hold_duration_s": (end - begin) / 30}
                                 for frame, begin, end in zip(captured.frames, boundaries, boundaries[1:])],
        }
        self.write_manifest()
        self.ffmpeg = tools(self.root / "tools")
        self.destination = self.root / "delivery"
        self.commands = []
        self.compressed = b"compressed"
        self.probe = {
            "streams": [{"codec_type": "video", "codec_name": "h264", "pix_fmt": "yuv420p",
                         "width": 2, "height": 2, "nb_read_frames": "18",
                         "avg_frame_rate": "30/1", "r_frame_rate": "30/1",
                         "time_base": "1/15360", "duration_ts": 9216, "start_pts": 0}],
            "format": {"duration": "0.600000"},
        }

    def write_manifest(self):
        (self.source / "manifest.json").write_text(json.dumps(self.manifest))

    def media(self, command, **kwargs):
        self.commands.append(command)
        if Path(command[0]).name == "ffprobe":
            kwargs["stdout"].write(json.dumps(self.probe).encode())
        elif command[-1].endswith("movie.mp4"):
            Path(command[-1]).write_bytes(self.compressed)
        elif command[-1].endswith("poster.png"):
            Path(command[-1]).write_bytes(png(110))
        return subprocess.CompletedProcess(command, 0)

    def invoke(self, max_bytes=10_000_000, poster_frame=7):
        # Shared process/probe primitives are reused from brand; its overlay path
        # must never execute for a native publication copy.
        with patch("viewer.video.brand.subprocess.run", side_effect=self.media):
            return export_video(self.source, self.destination, self.ffmpeg, poster_frame, max_bytes)

    def receipt(self):
        return json.loads((self.destination / "manifest.json").read_text())

    def test_small_movie_is_copied_exactly_with_original_physics_claims(self):
        movie = self.invoke()
        value = self.receipt()
        self.assertEqual(movie.read_bytes(), self.movie.read_bytes())
        self.assertEqual(value["schema"], "robodyna.publication_video.v1")
        self.assertTrue(value["complete"])
        self.assertTrue(value["full_decode_passed"])
        self.assertTrue(value["poster_decode_passed"])
        self.assertFalse(value["overlay_applied"])
        self.assertFalse(value["simulation_executed"])
        self.assertFalse(value["frame_cadence_changed"])
        self.assertEqual(value["encoding"], {"operation": "byte_copy", "lossy_reencode": False, "crf": None})
        self.assertIsNone(value["encode_command"])
        self.assertEqual(value["source_capture_metadata"], self.metadata)
        self.assertFalse(value["source_capture_metadata"]["input_horizon_complete"])
        self.assertEqual(value["video"]["sha256"], value["source_movie"]["sha256"])
        self.assertEqual(value["source_manifest"]["sha256"], sha256_file(self.source / "manifest.json"))
        self.assertEqual(value["poster"]["video_frame"], 7)
        self.assertEqual((value["video"]["frames"], value["video"]["fps"], value["video"]["duration_s"]), (18, 30, .6))
        self.assertFalse(any("-c:v" in command for command in self.commands))

    def test_large_movie_uses_only_one_crf22_compression_without_scene_filters(self):
        original = self.movie.read_bytes()
        self.invoke(max_bytes=16)
        value = self.receipt()
        command = value["encode_command"]
        self.assertEqual(value["encoding"], {"operation": "crf22_reencode", "lossy_reencode": True, "crf": 22})
        self.assertEqual(value["maximum_video_bytes"], 16)
        self.assertLessEqual(value["video"]["bytes"], 16)
        self.assertEqual(command[command.index("-crf") + 1], "22")
        self.assertEqual(command[command.index("-vsync") + 1], "0")
        for forbidden in ("-vf", "-filter_complex", "-r", "-s", "-frames:v", "-t"):
            self.assertNotIn(forbidden, command)
        self.assertEqual(sum("-c:v" in item for item in self.commands), 1)
        for item in self.commands:
            for i, option in enumerate(item):
                if option in ("-threads", "-filter_threads"):
                    self.assertEqual(item[i + 1], "1")
        self.assertEqual(self.movie.read_bytes(), original)

    def test_still_oversize_rejects_without_a_success_manifest_or_quality_retry(self):
        self.compressed = b"still too large for the requested cap"
        with self.assertRaisesRegex(ValueError, "exceeds the byte limit"):
            self.invoke(max_bytes=16)
        self.assertEqual(sum("-c:v" in item for item in self.commands), 1)
        self.assertFalse((self.destination / "manifest.json").exists())

    def test_native_manifest_and_capture_corruption_reject_before_processes(self):
        changes = [lambda d: d.update(full_decode_passed=False),
                   lambda d: d.update(schema="robodyna.branded_video.v1"),
                   lambda d: d.update(interpolated_frames=True),
                   lambda d: d["video"].update(file="../movie.mp4"),
                   lambda d: d["video"].update(sha256="a" * 64),
                   lambda d: d["video"].update(duration_s=.61),
                   lambda d: d["video"].update(frames=True),
                   lambda d: d.update(capture_manifest_sha256="a" * 64),
                   lambda d: d["capture_metadata"].update(input_horizon_complete=True),
                   lambda d: d["recorded_samples"][0].update(video_frames=5)]
        original = copy.deepcopy(self.manifest)
        for change in changes:
            with self.subTest(change=change):
                self.manifest = copy.deepcopy(original)
                change(self.manifest)
                self.write_manifest()
                with self.assertRaises(ValueError):
                    self.invoke()
                self.assertFalse(self.destination.exists())
        self.assertEqual(self.commands, [])

    def test_changed_capture_png_rejects_before_any_media_process(self):
        (self.capture / "frame-000002.png").write_bytes(png(1))
        with self.assertRaisesRegex(ValueError, "PNG size or digest"):
            self.invoke()
        self.assertEqual(self.commands, [])
        self.assertFalse(self.destination.exists())

    def test_source_symlink_duplicate_json_and_existing_output_are_not_admitted(self):
        original = self.movie.read_bytes()
        self.movie.unlink()
        other = self.root / "other.mp4"
        other.write_bytes(original)
        self.movie.symlink_to(other)
        with self.assertRaisesRegex(ValueError, "regular file"):
            self.invoke()
        self.movie.unlink()
        self.movie.write_bytes(original)
        path = self.source / "manifest.json"
        path.write_text(path.read_text().replace('"full_decode_passed": true',
                                                '"full_decode_passed": true, "full_decode_passed": true'))
        with self.assertRaisesRegex(ValueError, "Duplicate JSON"):
            self.invoke()
        self.write_manifest()
        self.destination.mkdir()
        with self.assertRaises(FileExistsError):
            self.invoke()
        self.assertEqual(self.commands, [])

    def test_bad_limits_poster_or_output_inside_inputs_never_launch(self):
        for limit in (0, -1, True, 10_000_001, 10.0):
            with self.subTest(limit=limit), self.assertRaises(ValueError):
                self.invoke(max_bytes=limit)
        for frame in (-1, 18, True, 1.0):
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                self.invoke(poster_frame=frame)
        for path in (self.source / "nested", self.capture / "nested"):
            self.destination = path
            with self.assertRaisesRegex(ValueError, "outside its preserved inputs"):
                self.invoke()
            self.assertFalse(path.exists())
        self.assertEqual(self.commands, [])

    def test_source_timing_mismatch_prevents_copy_or_compression(self):
        self.probe["streams"][0]["duration_ts"] += 1
        with self.assertRaises(ValueError):
            self.invoke()
        self.assertEqual(len(self.commands), 1)
        self.assertFalse((self.destination / "movie.mp4").exists())

    def test_output_dimensions_frames_or_exact_track_time_must_match(self):
        original_media = self.media
        original_probe = copy.deepcopy(self.probe)
        for field, invalid in (("width", 4), ("nb_read_frames", "17"), ("duration_ts", 9217)):
            self.destination = self.root / field
            self.probe = copy.deepcopy(original_probe)
            def altered(command, **kwargs):
                if Path(command[0]).name == "ffprobe" and command[-1] != str(self.movie):
                    self.probe["streams"][0][field] = invalid
                return original_media(command, **kwargs)
            self.media = altered
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.invoke(max_bytes=16)
            self.assertFalse((self.destination / "manifest.json").exists())

    def test_decoder_failure_and_late_output_mutation_never_publish(self):
        original_media = self.media
        def failed(command, **kwargs):
            if "-f" in command and command[-1] == "-":
                raise subprocess.CalledProcessError(1, command)
            return original_media(command, **kwargs)
        self.media = failed
        with self.assertRaises(subprocess.CalledProcessError):
            self.invoke()
        self.assertFalse((self.destination / "manifest.json").exists())
        self.destination = self.root / "mutated-output"
        def mutated(command, **kwargs):
            result = original_media(command, **kwargs)
            if command[-1].endswith("poster.png"):
                (self.destination / "movie.mp4").write_bytes(b"changed after decode")
            return result
        self.media = mutated
        with self.assertRaisesRegex(ValueError, "changed during export"):
            self.invoke()
        self.assertFalse((self.destination / "manifest.json").exists())

    def test_late_capture_mutation_never_publishes(self):
        original_media = self.media
        def mutated(command, **kwargs):
            result = original_media(command, **kwargs)
            if command[-1].endswith("poster.png"):
                (self.capture / "frame-000002.png").write_bytes(png(11))
            return result
        self.media = mutated
        with self.assertRaisesRegex(ValueError, "PNG size or digest"):
            self.invoke()
        self.assertFalse((self.destination / "manifest.json").exists())

    def test_late_source_movie_manifest_or_tool_mutation_never_publishes(self):
        original_media = self.media
        for target in (self.movie, self.source / "manifest.json", self.ffmpeg):
            previous = target.read_bytes()
            self.destination = self.root / ("mutated-" + target.name.replace(".", "-"))
            def mutated(command, **kwargs):
                result = original_media(command, **kwargs)
                if command[-1].endswith("poster.png"):
                    target.write_bytes(b"changed during export")
                return result
            self.media = mutated
            with self.subTest(target=target), self.assertRaisesRegex(ValueError, "changed during export"):
                self.invoke()
            self.assertFalse((self.destination / "manifest.json").exists())
            target.write_bytes(previous)

    def test_cli_uses_existing_tools_and_explicit_publication_options(self):
        with patch("viewer.video.publication.export_video", return_value=Path("movie.mp4")) as export:
            with patch("sys.stdout", new_callable=io.StringIO):
                main(["native", "delivery", "--ffmpeg", "/pinned/ffmpeg", "--poster-frame", "7"])
            export.assert_called_once_with("native", "delivery", "/pinned/ffmpeg", 7, 10_000_000)


if __name__ == "__main__":
    unittest.main()
