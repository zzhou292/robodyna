#!/usr/bin/env python3
"""Authenticate the eleven retained bodies; no numerical execution."""
from pathlib import Path
import hashlib,json,os
if os.environ.get("TEST_SRCDIR") and os.environ.get("TEST_WORKSPACE"):
    root=Path(os.environ["TEST_SRCDIR"])/os.environ["TEST_WORKSPACE"]
else:
    root=Path(__file__).resolve().parents[3]
manifest=json.loads((root/"lib_src/math/fixed_integer/ExtractionManifest.json").read_text())
source=(root/"lib_src/math/FixedInteger.h").read_text()
assert len(manifest["functions"])==11
for row in manifest["functions"]:
    token=row["declaration_prefix"]
    begin=source.index(token)
    begin=source.index("{",begin);end=begin+1;depth=1
    while depth:
        depth+=(source[end]=="{")-(source[end]=="}");end+=1
    body=source[begin:end]
    # The only device portability changes: pure unsigned value min/max.
    body=body.replace("detail::Maximum(a.used, b.used)","std::max(a.used, b.used)")
    body=body.replace("detail::Minimum(kLimbs, a.used + b.used)","std::min<unsigned>(kLimbs, a.used + b.used)")
    assert hashlib.sha256(body.encode()).hexdigest()==row["body_sha256"],token
assert "unsigned __int128" in source
assert "std::uint64_t limbs[kLimbs]{};" in source
for forbidden in ("new ","malloc(","thread_local","getenv(","__CUDA_ARCH__","std::array"):
    assert forbidden not in source,forbidden
print("PASS eleven original integer bodies; only scalar min/max spelling changes")
