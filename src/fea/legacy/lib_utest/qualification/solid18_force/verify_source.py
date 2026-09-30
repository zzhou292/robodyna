#!/usr/bin/env python3
"""Reuse the original owners of adhesive geometry and the LAW36 source curve."""
from pathlib import Path
import runpy

root = Path(__file__).resolve().parent.parent
runpy.run_path(str(root/'solid18_reference/verify_source.py'),run_name='__main__')
runpy.run_path(str(root/'solid_law36_point/verify_source.py'),run_name='__main__')
print('Solid18 force selected source geometry/material identity PASS')
