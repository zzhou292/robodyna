#!/usr/bin/env python3
"""Authenticate the focused original self-contact source-selection boundary."""

import hashlib
import json
from pathlib import Path
import py_compile


here = Path(__file__).resolve().parent
root = here.parents[1]
raw = (here / "source-manifest.json").read_bytes()
assert hashlib.sha256(raw).hexdigest() == "707c5a3f4d9c92c4aea2753df67776d4ab9913005b4ff83dead311e437db989a"
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
for token in (
    "*CONTACT_AUTOMATIC_SINGLE_SURFACE",
    "*SET_PART_ADD",
    "*SET_PART_LIST",
    "part-set expansion contains a cycle",
    "selected part is duplicated",
):
    assert token in cards
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
