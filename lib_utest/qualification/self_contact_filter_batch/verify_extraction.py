#!/usr/bin/env python3
"""Pin extracted arithmetic to qualified 21941d20; no git/runtime dependency."""
import hashlib
import re
from pathlib import Path

def body(source, name):
    source = re.sub(r"//[^\n]*|/\*.*?\*/", "", source, flags=re.S)
    match = re.search(r"\b" + re.escape(name) + r"\s*\([^;]*?\)\s*noexcept\s*\{", source)
    if not match:
        raise RuntimeError("Missing predicate body: " + name)
    start = match.end() - 1
    depth = 0
    for at in range(start, len(source)):
        depth += (source[at] == "{") - (source[at] == "}")
        if depth == 0:
            return source[start:at + 1]
    raise RuntimeError("Unclosed predicate body: " + name)

def normalized(value):
    # Only approved host/device spelling and geometry-type adaptation.
    for before, after in (("CurrentFixedTriangle", "Triangle"),
                          ("std::nextafter", "::nextafter"),
                          ("std::isfinite", "ScalarFinite"),
                          ("std::fabs", "::fabs"),
                          ("std::min", "OrderedMin"), ("std::max", "OrderedMax"),
                          ("std::numeric_limits<double>::infinity()", "HUGE_VAL")):
        value = value.replace(before, after)
    value = re.sub(r"\s+", "", value)
    # The accepted wrapper previously called the fixed production prism wrapper.
    value = value.replace("CertifiedLinearFacetPrismSeparationImpl<true,false>(", "CertifiedLinearFacetPrismSeparation(")
    value = value.replace("&axis,&valid,nullptr)", "&axis,&valid)")
    return value

EXPECTED = {'Down': 'd00ed01fe76ae842a664f9a3945f8718889e443a622956766684ebada1225751',
 'Up': '340a8c160b64f6bb5b98a044bbeef6e373acf2f36bd3b0cecde848a79c140f9b',
 'Finite': '6c9032c16ec62ea290b28c2650b4840ff78f9580467ef85e9d048d2d81ae7af0',
 'SameGeometry': '2ce446ed41d71a7d2fe04b3b27a363865adacf32c1fe343cf38027b4afdfc9c0',
 'ProductInterval': '4976ce78e242b882639f7c35820917a857581eaaf866a74a8a4433341b5f2943',
 'AddInterval': '860638a6d2c5ebef5fe4b76774f66615e2c1fc4ae960790f4e718eff9b17682a',
 'DotInterval': 'a95ce0b00dad0b65e6294bea11c0f47ff6994232eeecfad76cc1fb7ecb8b80a2',
 'ProjectionBounds': 'a7ecbb054fcfbfd3c5fe35142404808d9ca71825c5dfdd5346d784f2699a71fa',
 'EdgeAxis': '7670f1cdc9696d303ab66a1b4e4703e0bf427a0433ef49dcceae641e05e4212d',
 'CrossAxis': 'd26a9da2a03c6477c19dcd241f5e947735af9ff9b427e2974b3f13b23c412fbc',
 'Difference': '0e903d7548427438016c36e5cfc82c6ccc82700860ba96bfd44ef33e4779d4f2',
 'FaceAxis': 'f243d9b197f620ba8b053609c5297852f75a264a33d78ca20c8461dcaa64a8aa',
 'VertexEdgeAxis': '70ad12a04c77dfa301ded663eeca56222d9f5924325e187f6e0fb99f320bc06d',
 'InflateProjection': '50e040c158101c12daa5c3aca232f49651bf142adb0deba46bbf20f7d4068a81',
 'AxisSeparates': '2891a4fbd4a094fa426f961639cfb0847f48ba2fdd1fa7c875140d0bf70013e9',
 'Component': 'dac55694d9085c767f486f4f015e2f95589d503ce154598a1da2985cd1b03aa4',
 'InflatedFacetBoundsSeparated': 'd9736f516384eea5b27131f3e17a082d011c63982c0f16fd0d4dc8f3a94d4c9f',
 'EndpointHullsShareVertex': '16f952f2085af9307f1db10c559a3edc72c11ef1a5a1638df712a38ccd0725d9',
 'CertifiedLinearFacetPrismSeparationImpl': '8672bb11c5716471608a4fe8afa534eab2d6f0c92878666283ebeedc978db6d2',
 'ClassifyAcceptedFacetPair': '7eb6797962cbc64b0e917e78daa5b00a44208d37420ab5dd8153eeb0c6b3684c'}
PUBLIC_SHA256 = 'aa5ac0e9fd9a04b44bc63b78b1c188b431d8969423353b7134c481c20e3db0cf'
root = Path(__file__).resolve().parents[3]
collision = root / "lib_src/collision"
shared = "\n".join((collision / "self_contact_filters" / name).read_text()
                   for name in ("Arithmetic.h", "Prism.h"))
for old_name, digest in EXPECTED.items():
    name = "ClassifyAcceptedFacetPairImpl" if old_name == "ClassifyAcceptedFacetPair" else old_name
    actual = hashlib.sha256(normalized(body(shared, name)).encode()).hexdigest()
    if actual != digest:
        raise RuntimeError("Qualified arithmetic body changed: " + old_name)
if hashlib.sha256((collision / "SelfContactFilterCertificates.h").read_bytes()).hexdigest() != PUBLIC_SHA256:
    raise RuntimeError("Public certificate interface changed")
for spelling in ("return b < a ? b : a", "return a < b ? b : a", "return std::isfinite(value)"):
    if spelling not in shared:
        raise RuntimeError("Arithmetic platform adapter changed: " + spelling)
batch = (collision / "self_contact_filters/Batch.cpp").read_text()
kernel = (collision / "self_contact_filters/Kernels.cu").read_text()
cmake = (collision / "SelfContactFilterBatch.cmake").read_text()
for flag in ("--fmad=false", "--ftz=false", "-fno-fast-math", "-ffp-contract=off"):
    if flag not in cmake:
        raise RuntimeError("Missing arithmetic build contract: " + flag)
if batch.count("cudaMalloc(") != 1 or "cudaMalloc(" in batch[batch.index("Report Batch::Upload("):]:
    raise RuntimeError("Batch allocation escaped startup")
for token in ("cudaStreamSynchronize(stream)", "state.complete = false", "CompatibleHostArithmetic()",
              "Outside(pairs.data", "pairs.data[i].first >= state.scene_count"):
    if token not in batch:
        raise RuntimeError("Missing bounded batch admission: " + token)
if "results[i] = result" not in kernel or "atomic" in kernel:
    raise RuntimeError("Batch results no longer have unique ordered writers")
print("shared filter arithmetic source identity and bounded batch source: PASS")
