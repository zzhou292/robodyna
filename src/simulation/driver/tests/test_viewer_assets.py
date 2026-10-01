"""Small asset identity/runfile tests; no viewer or graphics initialization."""

from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from src.simulation.driver.rendering.assets import ANCHOR, FONT, load_assets
from src.simulation.driver.rendering.config import load_tools, viewer_asset_arguments
from .fixture import digest, write


class ViewerAssetTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.data = self.root / "data"
        self.records = []
        for name in (ANCHOR, FONT, "vsg/textures/sky.ktx", "colormaps/test.txt"):
            path = self.data / name
            path.parent.mkdir(parents=True, exist_ok=True)
            content = (name + " asset identity fixture").encode()
            path.write_bytes(content)
            self.records.append(dict(file=name, bytes=len(content), sha256=digest(content)))
        self.spec = dict(anchor=dict(path="data/" + ANCHOR), files=self.records)

    def test_only_visualization_roots_are_authenticated(self):
        unrelated = self.data / "vehicle/unrelated.mesh"
        unrelated.parent.mkdir()
        unrelated.write_bytes(b"outside visualization inventory")
        directory, records = load_assets(self.spec, self.root)
        self.assertEqual(directory, str(self.data))
        self.assertEqual(set(records), {entry["file"] for entry in self.records})
        self.assertNotIn("vehicle/unrelated.mesh", records)

    def test_manifest_only_runfiles_needs_file_anchor_not_directory_entry(self):
        spec = dict(anchor=dict(runfile="_main/src/compatibility/chrono/data/" + ANCHOR), files=self.records)
        with patch("src.simulation.driver.rendering.assets.runtime_file", return_value=self.data / ANCHOR) as locate:
            directory, _ = load_assets(spec, self.root)
        locate.assert_called_once_with(spec["anchor"]["runfile"])
        self.assertEqual(directory, str(self.data))

    def test_missing_changed_and_unlisted_assets_reject(self):
        (self.data / FONT).write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "identity differs"):
            load_assets(self.spec, self.root)
        original = next(row for row in self.records if row["file"] == FONT)
        (self.data / FONT).write_bytes((FONT + " asset identity fixture").encode())
        self.assertEqual((self.data / FONT).stat().st_size, original["bytes"])
        (self.data / "vsg/unlisted.txt").write_text("unexpected")
        with self.assertRaisesRegex(ValueError, "incomplete or unexpected"):
            load_assets(self.spec, self.root)

    def test_duplicate_or_escaping_asset_names_reject(self):
        with self.assertRaisesRegex(ValueError, "duplicate"):
            load_assets(dict(anchor=self.spec["anchor"], files=[*self.records, self.records[0]]), self.root)
        altered = [{**self.records[0], "file": "../logo_chrono_alpha.png"}, *self.records[1:]]
        with self.assertRaisesRegex(ValueError, "declared visualization roots"):
            load_assets(dict(anchor=self.spec["anchor"], files=altered), self.root)

    def test_native_tools_forward_authenticated_data_and_legacy_tools_remain_unchanged(self):
        tools = dict(schema="robodyna.render_tools.v1", chrono_data=self.spec)
        for name in ("viewer", "ffmpeg", "ffprobe"):
            path = self.root / name
            content = (name + " fixture - not executed").encode()
            path.write_bytes(content)
            path.chmod(0o700)
            tools[name] = dict(path=name, sha256=digest(content))
        write(self.root / "tools.json", tools)
        selected, _ = load_tools(self.root / "tools.json")
        self.assertEqual(viewer_asset_arguments(selected["viewer"]), ["--chrono-data", str(self.data)])
        self.assertEqual(sum(name.startswith("asset:") for name in selected), len(self.records))
        del tools["chrono_data"]
        write(self.root / "tools.json", tools)
        legacy, _ = load_tools(self.root / "tools.json")
        self.assertEqual(viewer_asset_arguments(legacy["viewer"]), [])


if __name__ == "__main__":
    unittest.main()
