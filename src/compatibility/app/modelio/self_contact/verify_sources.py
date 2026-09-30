#!/usr/bin/env python3
"""Authenticate the focused original self-contact source-selection boundary."""

import hashlib
import json
from pathlib import Path
import py_compile


here = Path(__file__).resolve().parent
root = here.parents[1]
raw = (here / "source-manifest.json").read_bytes()
assert hashlib.sha256(raw).hexdigest() == "4fd104582c5cd0cd7524261f5d9f687a6e3d126d39c26053be7da1ab204961cc"
manifest = json.loads(raw)
for row in manifest["files"]:
    path = Path(row["path"])
    assert not path.is_absolute() and ".." not in path.parts
    data = (root / path).read_bytes()
    assert len(data) == row["bytes"], path
    assert hashlib.sha256(data).hexdigest() == row["sha256"], path

sources = (here / "Sources.cpp").read_text()
assert "ReadRequestedSources" in sources
assert "keyword_counts" in sources
assert "source_block_sha256" in sources
assert "member.find(" not in sources

cards = (here / "Cards.cpp").read_text()
assert "*CONTACT_AUTOMATIC_SINGLE_SURFACE" in cards
assert "part_sets::Read(draft.candidates, limits.blocks, limits.parts)" in cards
assert "part_sets::Find(" in cards and "part_sets::Expand(" in cards
assert "draft.data.selected_part_ids, set_sources, limits.parts" in cards
sets = (here / "PartSets.cpp").read_text()
for token in (
    "*SET_PART_ADD",
    "*SET_PART_LIST",
    "part-set expansion contains a cycle",
    "selected part is duplicated",
    "list_limits.group_members = member_cap",
    "sets.size() < set_cap",
    "ordered.size() < member_cap",
    "tied_shell::detail::ListIds",
):
    assert token in sets
assert "using SourceSet = part_sets::Set" in (here / "Internal.h").read_text()
assert '"${CMAKE_CURRENT_LIST_DIR}/PartSets.cpp"' in (here / "OriginalSelection.cmake").read_text()
assert "static_friction" in cards and "dynamic_friction" in cards
assert "decay_coefficient" in cards and "ignore_initial_penetration" in cards

census = (here / "Census.cpp").read_text()
for array in ("shells_records", "solids_records", "beams_records"):
    assert array in census
assert "selected_parts" in census and "excluded_parts" in census

owner = (here / "OriginalSelection.cpp").read_text()
order = [
    "Preflight(",
    "ReadSources(",
    "ResolveCards(",
    "BuildCensus(",
    "OwnedPayload(",
]
positions = [owner.index(token) for token in order]
assert positions == sorted(positions)

tests = (here / "tests/OriginalTest.cpp").read_text()
for name, expected in manifest["independent_census"].items():
    if name == "report":
        continue
    assert f"data.counts.{name}, {expected}u" in tests

py_compile.compile(
    str(here / "tests/actual_fixture.py"),
    cfile="/tmp/robo-self-contact-actual-fixture.pyc",
    doraise=True,
)
print(
    json.dumps(
        {
            "status": "passed",
            "records": len(manifest["files"]),
            "source_selection_only": True,
            "runtime_contact": False,
            "numerical_execution": False,
        }
    )
)
