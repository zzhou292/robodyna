import hashlib
from pathlib import Path
import tempfile
import unittest
import zipfile

from tools.dependencies.zip_sdk import extract_sdk


class ZipSdkTest(unittest.TestCase):
    def test_extracts_declared_bytes_and_preserves_prior_install(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / "sdk.zip"
            with zipfile.ZipFile(package, "w") as source:
                source.writestr("tools/compiler.dat", b"compiler identity fixture")
            manifest = {"bytes": package.stat().st_size, "sha256": hashlib.sha256(package.read_bytes()).hexdigest(),
                        "max_expanded_bytes": 1024}
            receipt = extract_sdk(manifest, package, root / "install")
            self.assertEqual((root / "install/tools/compiler.dat").read_bytes(), b"compiler identity fixture")
            self.assertEqual(len(receipt["files"]), 1)
            with self.assertRaisesRegex(ValueError, "preserve"):
                extract_sdk(manifest, package, root / "install")

    def test_traversal_is_rejected_before_output_creation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / "bad.zip"
            with zipfile.ZipFile(package, "w") as source:
                source.writestr("../outside", b"must not escape")
            manifest = {"bytes": package.stat().st_size, "sha256": hashlib.sha256(package.read_bytes()).hexdigest(),
                        "max_expanded_bytes": 1024}
            with self.assertRaisesRegex(ValueError, "Unsafe"):
                extract_sdk(manifest, package, root / "install")
            self.assertFalse((root / "install").exists())

    def test_expansion_limit_is_checked_before_writing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / "large.zip"
            with zipfile.ZipFile(package, "w", compression=zipfile.ZIP_DEFLATED) as source:
                source.writestr("large", b"x" * 1000)
            manifest = {"bytes": package.stat().st_size, "sha256": hashlib.sha256(package.read_bytes()).hexdigest(),
                        "max_expanded_bytes": 100}
            with self.assertRaisesRegex(ValueError, "expansion"):
                extract_sdk(manifest, package, root / "install")
            self.assertFalse((root / "install").exists())


if __name__ == "__main__":
    unittest.main()
