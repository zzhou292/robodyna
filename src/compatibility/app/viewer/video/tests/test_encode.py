import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from viewer.video.encode import encode_capture, hold_boundaries, main
from viewer.video.tests.fixtures import capture, tools


class InputPipe(io.BytesIO):
    def close(self):
        if not self.closed:
            self.written = self.getvalue()
        super().close()


class FakeProcess:
    def __init__(self, command, **kwargs):
        self.command = command
        self.stdin = InputPipe()
        self.code = None
        self.terminated = False
        Path(command[-1]).write_bytes(b"mock encoded movie")

    def poll(self):
        return self.code

    def wait(self, timeout=None):
        self.code = 0 if self.code is None else self.code
        return self.code

    def terminate(self):
        self.terminated = True
        self.code = -15


class EncodeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.source = self.directory / "capture"
        self.rows, self.metadata = capture(self.source)
        self.ffmpeg = tools(self.directory / "tools")
        self.output = self.directory / "video"
        self.processes = []
        self.commands = []
        self.probe = {"streams": [{"codec_type": "video", "codec_name": "h264",
                                  "pix_fmt": "yuv420p", "width": 2, "height": 2,
                                  "nb_read_frames": "18", "avg_frame_rate": "30/1",
                                  "r_frame_rate": "30/1"}],
                      "format": {"duration": "0.600000"}}

    def popen(self, command, **kwargs):
        process = FakeProcess(command, **kwargs)
        self.processes.append(process)
        return process

    def run_media(self, command, **kwargs):
        self.commands.append(command)
        if Path(command[0]).name == "ffprobe":
            kwargs["stdout"].write(json.dumps(self.probe).encode())
        return subprocess.CompletedProcess(command, 0)

    def invoke(self, rate=5):
        with patch("viewer.video.encode.subprocess.Popen", side_effect=self.popen):
            with patch("viewer.video.encode.subprocess.run", side_effect=self.run_media):
                return encode_capture(self.source, self.output, self.ffmpeg, rate, 30)

    def test_exact_png_holds_commands_and_verified_normal_receipt(self):
        movie = self.invoke()
        self.assertEqual(movie, self.output / "movie.mp4")
        expected = b"".join((self.source / row["file"]).read_bytes() * 6 for row in self.rows)
        self.assertEqual(self.processes[0].stdin.written, expected)
        command = self.processes[0].command
        self.assertEqual(command[command.index("-framerate") + 1], "30")
        self.assertEqual(command[command.index("-frames:v") + 1], "18")
        self.assertIn("-n", command)
        self.assertNotIn("-vf", command)
        for index, option in enumerate(command):
            if option in ("-threads", "-filter_threads"):
                self.assertEqual(command[index + 1], "1")
        self.assertIn("-count_frames", self.commands[0])
        self.assertIn("-xerror", self.commands[1])
        receipt = json.loads((self.output / "manifest.json").read_text())
        self.assertTrue(receipt["full_decode_passed"])
        self.assertEqual(receipt["video"]["frames"], 18)
        self.assertEqual([row["first_video_frame"] for row in receipt["recorded_samples"]], [0, 6, 12])
        self.assertEqual([row["accepted_time_s"] for row in receipt["recorded_samples"]],
                         [row["accepted_time_s"] for row in self.rows])
        self.assertEqual(receipt["capture_metadata"], self.metadata)

    def test_fractional_rate_and_recovered_metadata(self):
        self.source = self.directory / "recovered"
        self.rows, self.metadata = capture(self.source, recovered=True)
        self.probe["streams"][0]["nb_read_frames"] = "23"
        self.probe["format"]["duration"] = "0.766667"
        self.invoke(rate=4)
        receipt = json.loads((self.output / "manifest.json").read_text())
        self.assertEqual([row["video_frames"] for row in receipt["recorded_samples"]], [8, 7, 8])
        self.assertEqual(receipt["capture_metadata"], self.metadata)
        self.assertNotIn("input_archive_manifest", receipt["capture_metadata"])
        self.assertNotIn("input_horizon_complete", receipt["capture_metadata"])

    def test_invalid_rate_capture_or_existing_output_never_launches(self):
        for rate, fps in ((0, 30), (float("nan"), 30), (31, 30), (5, 0), (5, 30.0)):
            with self.subTest(rate=rate, fps=fps):
                with patch("viewer.video.encode.subprocess.Popen") as launch:
                    with self.assertRaises(ValueError):
                        encode_capture(self.source, self.output, self.ffmpeg, rate, fps)
                    launch.assert_not_called()
                    self.assertFalse(self.output.exists())
        self.output.mkdir()
        (self.output / "keep").write_text("unchanged")
        with patch("viewer.video.encode.subprocess.Popen") as launch:
            with self.assertRaises(FileExistsError):
                encode_capture(self.source, self.output, self.ffmpeg)
            launch.assert_not_called()
        self.assertEqual((self.output / "keep").read_text(), "unchanged")

    def test_probe_mismatch_and_decode_failure_do_not_publish_success(self):
        self.probe["streams"][0]["nb_read_frames"] = "17"
        with self.assertRaises(ValueError):
            self.invoke()
        self.assertFalse((self.output / "manifest.json").exists())
        self.assertEqual(len(self.commands), 1)
        self.output = self.directory / "decode-failure"
        self.probe["streams"][0]["nb_read_frames"] = "18"
        original_run = self.run_media
        def fail_decode(command, **kwargs):
            if "-xerror" in command:
                raise subprocess.CalledProcessError(1, command)
            return original_run(command, **kwargs)
        self.run_media = fail_decode
        with self.assertRaises(subprocess.CalledProcessError):
            self.invoke()
        self.assertFalse((self.output / "manifest.json").exists())

    def test_mutation_during_stream_stops_child_without_success_receipt(self):
        original_popen = self.popen
        def changed(command, **kwargs):
            image = self.source / self.rows[-1]["file"]
            data = bytearray(image.read_bytes())
            data[-1] ^= 1
            image.write_bytes(data)
            return original_popen(command, **kwargs)
        self.popen = changed
        with self.assertRaisesRegex(ValueError, "changed during encoding"):
            self.invoke()
        self.assertTrue(self.processes[0].terminated)
        self.assertFalse((self.output / "manifest.json").exists())
        self.assertEqual(self.commands, [])

    def test_encoder_failure_does_not_probe_or_publish(self):
        original_popen = self.popen
        def fail_encode(command, **kwargs):
            process = original_popen(command, **kwargs)
            process.code = 1
            return process
        self.popen = fail_encode
        with self.assertRaises(subprocess.CalledProcessError):
            self.invoke()
        self.assertEqual(len(self.processes), 1)
        self.assertEqual(self.commands, [])
        self.assertFalse((self.output / "manifest.json").exists())

    def test_late_capture_change_does_not_publish(self):
        original_run = self.run_media
        def change_metadata(command, **kwargs):
            if "-xerror" in command:
                path = self.source / "manifest.json"
                metadata = json.loads(path.read_text())
                metadata["input_stop_reason"] = "Changed after validation"
                path.write_text(json.dumps(metadata))
            return original_run(command, **kwargs)
        self.run_media = change_metadata
        with self.assertRaisesRegex(ValueError, "changed during encoding"):
            self.invoke()
        self.assertFalse((self.output / "manifest.json").exists())

    def test_cli_and_hold_plan(self):
        self.assertEqual(hold_boundaries(41, 5, 30)[-1], 246)
        with patch("viewer.video.encode.encode_capture", return_value=Path("movie.mp4")) as encode:
            with patch("sys.stdout", new_callable=io.StringIO):
                main(["capture", "output", "--ffmpeg", "/pinned/ffmpeg",
                      "--samples-per-second", "5", "--output-fps", "30"])
            encode.assert_called_once_with("capture", "output", "/pinned/ffmpeg", 5.0, 30)


if __name__ == "__main__":
    unittest.main()
