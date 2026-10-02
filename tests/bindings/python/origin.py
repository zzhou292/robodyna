"""Authenticate imported runtime files across Bazel's per-file runfile symlinks."""

from pathlib import Path


def require_declared_origin(module, manifest, document, relative_initializer):
    """Require the imported file itself, not its unresolved runfile directory."""
    manifest = Path(manifest)
    candidates = [manifest.parent / relative / relative_initializer
                  for relative in document["runtime_python_roots"]]
    declared = {path.resolve(strict=True) for path in candidates if path.is_file()}
    actual = Path(module.__file__).resolve(strict=True)
    if actual not in declared:
        raise RuntimeError(module.__name__ + " came from an undeclared runtime file")
