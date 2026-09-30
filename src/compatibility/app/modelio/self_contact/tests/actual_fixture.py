"""Authenticate original contact members, then reuse the canonical source fixture."""

import hashlib
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import zipfile


canonical, scope, binary = map(Path, sys.argv[1:4])
test_filter = (
    sys.argv[4]
    if len(sys.argv) > 4
    else "OriginalSelfContactSelectionActual.*"
)
prefix = "2010-toyota-yaris-coarse-v1l/"
members = {
    "set-yaris-coarse-v1l.key": (
        44991,
        "b93d5370a899f6f70299ea61cd55142c1f8b765b8ab7f9ac979d078486028929",
        ("ROBO_SELF_CONTACT_AUX_MEMBER", "ROBO_TIED_AUX_MEMBER"),
    ),
    "combine.key": (
        10577,
        "3e0137cd8c569a71a4281cc307549dc2f20772bc67eac0658ae73682dbe242a2",
        ("ROBO_SELF_CONTACT_COMBINE_MEMBER",),
    ),
    "wall.key": (
        10604,
        "ef02a4701b37d27cec81b1f9a02ab555f55ac61f68b070e8b0c18dc23b1d5155",
        ("ROBO_TIED_WALL_MEMBER",),
    ),
}

with tempfile.TemporaryDirectory(prefix="robo-self-contact-source-") as temporary:
    temporary = Path(temporary)
    environment = dict(os.environ)
    with zipfile.ZipFile(canonical / "source_model.zip") as archive:
        for name, (size, digest, variables) in members.items():
            source_name = prefix + name
            if archive.namelist().count(source_name) != 1:
                raise RuntimeError(f"missing or duplicate original {name}")
            info = archive.getinfo(source_name)
            if info.file_size != size:
                raise RuntimeError(f"original {name} extent changed")
            data = archive.read(source_name)
            if hashlib.sha256(data).hexdigest() != digest:
                raise RuntimeError(f"original {name} identity changed")
            target = temporary / name
            with target.open("xb") as stream:
                stream.write(data)
            for variable in variables:
                environment[variable] = str(target)

    helper = (
        Path(__file__).resolve().parents[3]
        / "output/full_shell/static_bundle/tests/actual_source_fixture.py"
    )
    result = subprocess.run(
        [
            sys.executable,
            "-B",
            str(helper),
            str(canonical),
            str(scope),
            str(binary),
            test_filter,
        ],
        env=environment,
        check=False,
    )
    raise SystemExit(result.returncode)
