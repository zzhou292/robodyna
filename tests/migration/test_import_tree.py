"""Exercise source import against small real Git repositories."""

import fcntl
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from tools.migration.import_tree import import_tree, validate_prefix


def git(path, *arguments):
    return subprocess.check_output(
        ["git", "-C", str(path), *arguments], text=True,
        stderr=subprocess.DEVNULL,
    ).strip()


def initialize(path):
    path.mkdir()
    git(path, "init", "--quiet")
    git(path, "config", "user.name", "Migration Test")
    git(path, "config", "user.email", "migration-test@example.invalid")
    git(path, "config", "commit.gpgsign", "false")


class ImportTreeTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        base = Path(self.temporary.name)
        self.source, self.destination = base / "source", base / "destination"
        self.lock = base / "author.lock"
        initialize(self.source)
        initialize(self.destination)
        (self.source / "binary.dat").write_bytes(b"\x00\xff\x13preserved\n")
        script = self.source / "program"
        script.write_text("#!/bin/sh\nexit 0\n")
        script.chmod(0o755)
        (self.source / "alias").symlink_to("binary.dat")
        git(self.source, "add", ".")
        git(self.source, "commit", "--quiet", "-m", "Source fixture")
        self.commit = git(self.source, "rev-parse", "HEAD")
        (self.destination / "README.md").write_text("Destination\n")
        git(self.destination, "add", ".")
        git(self.destination, "commit", "--quiet", "-m", "Destination fixture")
        self.original = git(self.destination, "rev-parse", "HEAD")
        git(self.destination, "fetch", "--quiet", str(self.source),
            self.commit + ":refs/import/source")

    def perform(self, **overrides):
        values = dict(repository=self.destination, source_ref="refs/import/source",
                      expected_commit=self.commit, prefix="owned/source",
                      author_lock=self.lock)
        values.update(overrides)
        return import_tree(**values)

    def assert_unchanged(self):
        self.assertEqual(git(self.destination, "rev-parse", "HEAD"), self.original)
        self.assertFalse((self.destination / ".git/MERGE_HEAD").exists())

    def test_tree_modes_history_and_untracked_overlay_are_preserved(self):
        overlay = self.destination / "owned/source/BUILD.bazel"
        overlay.parent.mkdir(parents=True)
        overlay.write_text("# untracked migration declaration\n")
        result = self.perform()
        self.assertEqual(git(self.destination, "rev-parse", "HEAD:owned/source"),
                         git(self.source, "rev-parse", "HEAD^{tree}"))
        self.assertTrue(result["source_history_reachable"])
        self.assertEqual((overlay.parent / "binary.dat").read_bytes(),
                         (self.source / "binary.dat").read_bytes())
        self.assertEqual(os.readlink(overlay.parent / "alias"), "binary.dat")
        self.assertTrue(os.access(overlay.parent / "program", os.X_OK))
        self.assertEqual(overlay.read_text(), "# untracked migration declaration\n")
        self.assertEqual(git(self.source, "rev-parse", "HEAD"), self.commit)
        self.assertEqual(git(self.source, "status", "--porcelain"), "")

    def test_wrong_source_pin_fails_before_mutation(self):
        with self.assertRaises(ValueError):
            self.perform(expected_commit="0" * 40)
        self.assert_unchanged()

    def test_dirty_destination_is_not_overwritten(self):
        path = self.destination / "README.md"
        path.write_text("Uncommitted user work\n")
        with self.assertRaises(subprocess.CalledProcessError):
            self.perform()
        self.assert_unchanged()
        self.assertEqual(path.read_text(), "Uncommitted user work\n")

    def test_occupied_prefix_is_not_replaced(self):
        with self.assertRaises(ValueError):
            self.perform(prefix="README.md")
        self.assert_unchanged()

    def test_held_author_lock_is_not_bypassed(self):
        with self.lock.open("a") as held:
            fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaises(BlockingIOError):
                self.perform()
        self.assert_unchanged()

    def test_unsafe_prefixes_reject(self):
        for prefix in ("", ".", "../outside", "/absolute", "a/../b", ".git/x", "a//b"):
            with self.subTest(prefix=prefix), self.assertRaises(ValueError):
                validate_prefix(prefix)


if __name__ == "__main__":
    unittest.main()
