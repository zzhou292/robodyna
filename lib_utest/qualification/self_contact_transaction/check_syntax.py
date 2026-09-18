#!/usr/bin/env python3
"""Compile the public and implementation transaction shapes without NVCC."""
import argparse
import subprocess
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--compiler", required=True)
parser.add_argument("--output", required=True)
args = parser.parse_args()

root = Path(__file__).resolve().parents[3]
output = Path(args.output)
output.mkdir(parents=True, exist_ok=True)
probe = output / "PublicProbe.cpp"
probe.write_text(
    '#include "lib_src/collision/SelfContactTransaction.h"\n'
    "#include <type_traits>\n"
    "using namespace tlfea::contact;\n"
    "static_assert(!std::is_aggregate_v<SelfContactTransactionReceipt>);\n"
    "static_assert(!std::is_copy_constructible_v<SelfContactTransaction>);\n"
    "int main(){ SelfContactTransactionReceipt r; "
    "return r.scratch_receipts().self_contact != nullptr; }\n"
)

common = [
    args.compiler,
    "-std=c++17",
    "-fno-fast-math",
    "-ffp-contract=off",
    "-x",
    "c++",
    "-I",
    str(root),
    "-I",
    "/usr/local/cuda/include",
    "-I",
    "/usr/include/eigen3",
    "-fsyntax-only",
]
sources = [
    probe,
    root / "lib_src/collision/SelfContactFilterCertificates.cpp",
    root / "lib_src/collision/self_contact_transaction/Arena.cpp",
    root / "lib_src/collision/self_contact_transaction/Limits.cpp",
    root / "lib_src/collision/self_contact_transaction/Layout.cpp",
    root / "lib_src/collision/self_contact_transaction/Values.cpp",
    root / "lib_src/collision/self_contact_transaction/LocalContact.cpp",
    root / "lib_src/collision/self_contact_transaction/TranslatedLocal.cpp",
    root / "lib_src/collision/self_contact_transaction/CrossingBatch.cpp",
    root / "lib_src/collision/self_contact_transaction/RigidSweep.cpp",
    root / "lib_src/collision/self_contact_transaction/Streaming.cpp",
    root / "lib_src/collision/self_contact_transaction/TaskMask.cpp",
    root / "lib_src/collision/self_contact_transaction/Source.cpp",
    root / "lib_src/collision/self_contact_transaction/Qualification.cpp",
    root / "lib_src/collision/self_contact_transaction/CandidateFailureCapture.cpp",
    root / "lib_src/collision/self_contact_transaction/Transaction.cpp",
    root / "lib_src/collision/self_contact_transaction/Candidate.cpp",
    root / "lib_src/collision/self_contact_transaction/CandidateExclusions.cpp",
    root / "lib_src/collision/SelfContactBroadphase.cpp",
    root / "lib_src/solvers/NodalOwnerStream.cpp",
    root / "lib_src/collision/self_contact_force/Initialize.cpp",
    root / "lib_utest/qualification/self_contact_transaction/CudaTest.cu",
]
for source in sources:
    subprocess.run([*common, str(source)], check=True)
print("fixed self-contact transaction syntax: PASS")
