"""Exact final-copy extraction shared by historical caller source proofs."""
from pathlib import Path

HERE = Path(__file__).resolve().parent
REFERENCE = (HERE/'reference/ExplicitNodalCinStep.cu').read_text()
START = REFERENCE.index('  if (capture.node) {\n    for (std::uint32_t node')
END = REFERENCE.index('\n}\n} // namespace', START)
CAPTURE = REFERENCE[START:END]

def without_capture(value):
    assert value.count(CAPTURE) == 1, 'expected exactly the unchanged frozen copy loop'
    return value.replace(CAPTURE, '')
