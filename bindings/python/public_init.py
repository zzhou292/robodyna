"""Robodyna public Python package backed by the retained, shared native modules."""

import sys as _sys
import pychrono as _compatibility

for _name in _compatibility.__all__:
    globals()[_name] = getattr(_compatibility, _name)
for _name, _module in _compatibility._module_objects.items():
    globals()[_name] = _module
    _sys.modules[__name__ + "." + _name] = _module

__all__ = list(_compatibility.__all__)
