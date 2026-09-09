"""Reuse existing importer utilities without changing their CLI import contract.

The older standalone tools use sibling absolute imports. Temporarily expose
that directory while importing them, then restore the search path. Importing
this package does not parse a model or execute either tool's guarded main().
"""
from pathlib import Path
import sys

_tools = str(Path(__file__).resolve().parents[1] / 'tools')
sys.path.insert(0, _tools)
try:
    from import_yaris_vehicle import fields, file_sha256
    from import_yaris_wall import WallImportError, require, sha256
finally:
    sys.path.remove(_tools)

__all__ = ['fields', 'file_sha256', 'WallImportError', 'require', 'sha256']
