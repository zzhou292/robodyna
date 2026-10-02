"""Reproduce per-file runfile links and reject equal-content ambient copies."""

from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from tests.bindings.python.origin import require_declared_origin


class RuntimeOriginTest(unittest.TestCase):
    def test_individual_file_links_preserve_declared_origin(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            installed = root / "sdk/numpy/__init__.py"
            installed.parent.mkdir(parents=True)
            installed.write_text("VERSION = 'test'\n")
            runfile = root / "runfiles/runtime/numpy/__init__.py"
            runfile.parent.mkdir(parents=True)
            runfile.symlink_to(installed)
            manifest = root / "runfiles/package/package.json"
            manifest.parent.mkdir()
            document = {"runtime_python_roots": ["../runtime"]}
            module = SimpleNamespace(__name__="numpy", __file__=str(installed))
            require_declared_origin(module, manifest, document, "numpy/__init__.py")
            ambient = root / "ambient/numpy/__init__.py"
            ambient.parent.mkdir(parents=True)
            ambient.write_bytes(installed.read_bytes())
            module.__file__ = str(ambient)
            with self.assertRaisesRegex(RuntimeError, "undeclared runtime file"):
                require_declared_origin(module, manifest, document, "numpy/__init__.py")


if __name__ == "__main__":
    unittest.main()
