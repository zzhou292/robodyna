import tempfile
import hashlib
import io
import tarfile
from pathlib import Path
import unittest

from tools.dependencies.cmake_sdk import check_cache, tree_identity, extract_source


class CmakeSdkTest(unittest.TestCase):
    def test_pinned_embedded_source_never_overwrites_existing_content(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            archive = root / "source.tar.gz"
            with tarfile.open(archive, "w:gz") as stream:
                header = tarfile.TarInfo("component/source.cpp")
                data = b"retained source\n"
                header.size = len(data)
                stream.addfile(header, io.BytesIO(data))
            identity = {"bytes": archive.stat().st_size, "sha256": hashlib.sha256(archive.read_bytes()).hexdigest()}
            destination = root / "owned" / "thirdparty" / "component"
            destination.mkdir(parents=True)
            extract_source(archive, identity, destination)
            self.assertEqual((destination / "source.cpp").read_bytes(), data)
            with self.assertRaisesRegex(ValueError, "new or an empty"):
                extract_source(archive, identity, destination)

    def test_profile_cache_rejects_an_unrequested_library_type(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "CMakeCache.txt"
            path.write_text("// note\nBUILD_LIBRARY_TYPE:STRING=Shared\nCMAKE_BUILD_TYPE:STRING=Release\n")
            self.assertEqual(check_cache(path, {"BUILD_LIBRARY_TYPE": "Shared"}), {"BUILD_LIBRARY_TYPE": "Shared"})
            with self.assertRaisesRegex(ValueError, "cache mismatch"):
                check_cache(path, {"BUILD_LIBRARY_TYPE": "Static"})

    def test_source_identity_changes_for_edits_and_dangling_links_reject(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.cpp"
            source.write_bytes(b"original source")
            before = tree_identity(root)
            source.write_bytes(b"edited source")
            self.assertNotEqual(tree_identity(root), before)
            (root / "missing.h").symlink_to(root / "absent.h")
            with self.assertRaisesRegex(ValueError, "dangling symlink"):
                tree_identity(root)


if __name__ == "__main__":
    unittest.main()
