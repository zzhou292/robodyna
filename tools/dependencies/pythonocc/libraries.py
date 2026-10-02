"""Order the admitted OCCT DSO closure without importing or loading libraries."""


def dependency_order(libraries, roots):
    """Return required admitted SONAMEs with dependencies first; reject gaps/cycles."""
    required = set()
    pending = list(roots)
    while pending:
        name = pending.pop()
        if name in required:
            continue
        if name not in libraries:
            raise ValueError("Required OCCT library is not admitted: " + name)
        required.add(name)
        pending.extend(item for item in libraries[name]["needed"] if item.startswith("libTK"))
    ordered = []
    remaining = set(required)
    while remaining:
        ready = sorted(name for name in remaining
                       if not set(libraries[name]["needed"]).intersection(remaining))
        if not ready:
            raise ValueError("Required OCCT library dependency cycle: " + ", ".join(sorted(remaining)))
        ordered.extend(ready)
        remaining.difference_update(ready)
    return ordered
