"""Read the literal library subset used by the frozen application source.

This is an audit reader, not a CMake interpreter or a build-time translator.
Selected targets must have literal source/dependency lists in their owning file.
"""

from pathlib import Path
import re
import shlex

_COMMAND = re.compile(
    r"^\s*(add_library|target_sources|target_link_libraries|target_compile_options)\s*\(",
    re.MULTILINE | re.IGNORECASE,
)
_SCOPES = {"PUBLIC", "PRIVATE", "INTERFACE"}


def commands(text):
    for match in _COMMAND.finditer(text):
        start = cursor = match.end()
        depth, quoted, escaped = 1, False, False
        while cursor < len(text) and depth:
            char = text[cursor]
            if char == '"' and not escaped:
                quoted = not quoted
            if not quoted:
                depth += (char == "(") - (char == ")")
            escaped = char == "\\" and not escaped
            cursor += 1
        if depth:
            raise ValueError("Unclosed literal CMake command")
        yield match[1].lower(), shlex.split(text[start:cursor - 1], comments=True)


def library_source(text, name, owner):
    """Return normalized literal sources, dependencies and private C++ options."""
    sources, dependencies, copts = [], [], []
    definitions = 0
    for command, args in commands(text):
        if not args or args[0] != name:
            continue
        values = [value for value in args[1:] if value not in _SCOPES]
        if command == "add_library":
            definitions += 1
            if not values or values[0] != "STATIC":
                raise ValueError(f"Expected selected STATIC library: {name}")
            sources.extend(values[1:])
        elif command == "target_sources":
            sources.extend(values)
        elif command == "target_link_libraries":
            dependencies.extend(values)
        elif command == "target_compile_options":
            copts.extend(values)
    if definitions != 1:
        raise ValueError(f"Expected exactly one source definition of {name}")
    normalized = []
    for source in sources:
        rooted = source.startswith(("${CMAKE_CURRENT_LIST_DIR}", "${CMAKE_CURRENT_SOURCE_DIR}"))
        source = source.replace("${CMAKE_CURRENT_LIST_DIR}", str(Path(owner).parent))
        source = source.replace("${CMAKE_CURRENT_SOURCE_DIR}", str(Path(owner).parent))
        if "${" in source or "$<" in source:
            raise ValueError(f"Unresolved CMake source operand: {source}")
        if not rooted:
            source = str(Path(owner).parent / source)
        # Normalize inside a fictitious absolute root, then reject upward escape.
        path = (Path("/app") / source).resolve()
        if not path.is_relative_to("/app"):
            raise ValueError(f"Source escapes application: {source}")
        normalized.append(str(path.relative_to("/app")))
    return normalized, dependencies, copts
