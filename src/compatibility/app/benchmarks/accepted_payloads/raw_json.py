"""Compatibility import for shared bounded, exact-spelling JSON reads."""
if __package__:
    from output.json_object import read_object
else:
    # Preserve the historically supported direct compare.py invocation from
    # any directory, without adding mutable search paths or importing a viewer.
    import importlib.util
    from pathlib import Path
    _path = Path(__file__).resolve().parents[2] / "output/json_object.py"
    _spec = importlib.util.spec_from_file_location("robo_output_json_object", _path)
    _module = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_module)
    read_object = _module.read_object
