"""Branding admission, exact timing and provenance; no media subprocesses."""

import copy
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from viewer.file_integrity import sha256_file
from viewer.video.brand import brand_video
from viewer.video.tests.fixtures import png, tools


class BrandTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / "source"
        self.source.mkdir()
        self.original = self.source / "movie.mp4"
        self.original.write_bytes(b"original authenticated movie")
        self.source_manifest = {
            "schema": "robo_dyna.chrono_capture_video.v1", "full_decode_passed": True,
            "video": {"file": "movie.mp4", "frames": 182, "fps": 30,
                      "bytes": self.original.stat().st_size,
                      "sha256": sha256_file(self.original)},
            "capture_metadata": {"width": 1280, "height": 800,
                                 "schema": "robodyna.chrono_live_capture.v1",
                                 "final_time_s": 6.0, "physics_backend": "chrono_cpu"},
        }
        self.write_manifest()
        self.logo = self.root / "logo.png"
        self.logo.write_bytes(png(128))
        self.ffmpeg = tools(self.root / "tools")
        self.destination = self.root / "branded"
        self.commands = []
        self.probe = {
            "streams": [{"codec_type": "video", "codec_name": "h264", "pix_fmt": "yuv420p",
                         "width": 1280, "height": 800, "nb_read_frames": "182",
                         "avg_frame_rate": "30/1", "r_frame_rate": "30/1",
                         "time_base": "1/15360", "duration_ts": 93184, "start_pts": 0}],
            "format": {"duration": "6.067000"},
        }

    def write_manifest(self):
        (self.source / "manifest.json").write_text(json.dumps(self.source_manifest))

    def media(self, command, **kwargs):
        self.commands.append(command)
        if Path(command[0]).name == "ffprobe":
            kwargs["stdout"].write(json.dumps(self.probe).encode())
        elif command[-1].endswith("movie.mp4"):
            Path(command[-1]).write_bytes(b"branded movie")
        elif command[-1].endswith("poster.png"):
            Path(command[-1]).write_bytes(png(127))
        return subprocess.CompletedProcess(command, 0)

    def invoke(self, frame=50):
        with patch("viewer.video.brand.subprocess.run", side_effect=self.media):
            return brand_video(self.source, self.destination, self.logo, self.ffmpeg, frame)

    def test_exact_cadence_declared_panel_and_independent_receipt(self):
        original = self.original.read_bytes()
        movie = self.invoke()
        result = json.loads((self.destination / "manifest.json").read_text())
        self.assertEqual(result["schema"], "robodyna.branded_video.v1")
        self.assertFalse(result["simulation_executed"])
        self.assertFalse(result["frame_cadence_changed"])
        self.assertTrue(result["full_decode_passed"])
        self.assertTrue(result["poster_decode_passed"])
        self.assertEqual(result["video"]["frames"], 182)
        self.assertEqual(result["video"]["fps"], 30)
        self.assertEqual(result["poster"]["video_frame"], 50)
        self.assertEqual(result["source_capture_metadata"], self.source_manifest["capture_metadata"])
        self.assertEqual(result["source_movie"]["sha256"], sha256_file(self.original))
        self.assertEqual(self.original.read_bytes(), original)
        self.assertEqual(movie, self.destination / "movie.mp4")
        command = result["encode_command"]
        self.assertNotIn("-r", command)
        self.assertNotIn("-frames:v", command)
        self.assertEqual(command[command.index("-vsync") + 1], "0")
        self.assertEqual(command[command.index("-crf") + 1], "22")
        self.assertIn("drawbox=x=iw-248:y=8:w=240:h=106", command[command.index("-filter_complex") + 1])
        for item in self.commands:
            for index, option in enumerate(item):
                if option in ("-threads", "-filter_threads", "-filter_complex_threads"):
                    self.assertEqual(item[index + 1], "1")
        self.assertEqual(sum("-xerror" in item for item in self.commands), 3)

    def test_unverified_mutated_or_traversing_source_never_launches(self):
        for mutate in (lambda: self.source_manifest.update(full_decode_passed=False),
                       lambda: self.source_manifest["video"].update(file="../movie.mp4"),
                       lambda: self.original.write_bytes(b"changed")):
            with self.subTest(mutate=mutate):
                original = copy.deepcopy(self.source_manifest)
                mutate()
                self.write_manifest()
                with patch("viewer.video.brand.subprocess.run") as launch:
                    with self.assertRaises(ValueError):
                        brand_video(self.source, self.destination, self.logo, self.ffmpeg, 50)
                    launch.assert_not_called()
                self.assertFalse(self.destination.exists())
                self.source_manifest = original
                self.write_manifest()

    def test_create_only_and_invalid_poster_never_launch(self):
        for frame in (-1, 182, True, 0.0):
            with self.subTest(frame=frame):
                with self.assertRaises(ValueError):
                    self.invoke(frame)
        self.assertEqual(self.commands, [])
        self.destination.mkdir()
        with self.assertRaises(FileExistsError):
            self.invoke()
        self.assertEqual(self.commands, [])

    def test_source_probe_mismatch_fails_before_encode(self):
        self.probe["streams"][0]["duration_ts"] += 1
        with self.assertRaises(ValueError):
            self.invoke()
        self.assertEqual(len(self.commands), 1)
        self.assertFalse((self.destination / "manifest.json").exists())

    def test_output_frame_or_timing_changes_never_publish(self):
        original_media = self.media
        def changed(command, **kwargs):
            if Path(command[0]).name == "ffprobe" and command[-1] != str(self.original):
                self.probe["streams"][0]["nb_read_frames"] = "181"
            return original_media(command, **kwargs)
        self.media = changed
        with self.assertRaises(ValueError):
            self.invoke()
        self.assertFalse((self.destination / "manifest.json").exists())

    def test_decode_failure_never_publishes(self):
        original_media = self.media
        def failed(command, **kwargs):
            if "-xerror" in command:
                raise subprocess.CalledProcessError(1, command)
            return original_media(command, **kwargs)
        self.media = failed
        with self.assertRaises(subprocess.CalledProcessError):
            self.invoke()
        self.assertFalse((self.destination / "manifest.json").exists())

    def test_late_input_mutation_never_publishes(self):
        original_media = self.media
        def changed(command, **kwargs):
            result = original_media(command, **kwargs)
            if command[-1].endswith("poster.png"):
                self.logo.write_bytes(png(14))
            return result
        self.media = changed
        with self.assertRaisesRegex(ValueError, "changed during branding"):
            self.invoke()
        self.assertFalse((self.destination / "manifest.json").exists())


if __name__ == "__main__":
    unittest.main()
