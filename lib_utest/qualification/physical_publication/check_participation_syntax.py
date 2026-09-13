#!/usr/bin/env python3
"""Compile the host-only participation implementation without CUDA language."""
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
    '#include "lib_src/elements/ShellBatchPublication.h"\n'
    '#include "lib_src/collision/NodalWallMappedContact.h"\n'
    "#include <type_traits>\n"
    "using namespace tl::fea;\n"
    "static_assert(!std::is_aggregate_v<"
    "ShellPhysicalScratchParticipationReceipt>);\n"
    "static_assert(!std::is_aggregate_v<"
    "tlfea::contact::NodalWallMappedTransactionReceipt>);\n"
    "static_assert(static_cast<unsigned>("
    "ShellPhysicalScratchContributorKind::MappedWall)==0);\n"
    "static_assert(static_cast<unsigned>("
    "ShellPhysicalScratchContributorKind::SelfContact)==1);\n"
    "int main(){return sizeof(ShellPhysicalScratchParticipation)==0;}\n"
)
common = [
    args.compiler,
    "-std=c++17",
    "-fno-fast-math",
    "-ffp-contract=off",
    "-I",
    str(root),
    "-I",
    "/usr/local/cuda/include",
    "-I",
    "/usr/include/eigen3",
    "-fsyntax-only",
]
for source in (
    probe,
    root / "lib_src/elements/ShellBatchPublication.cpp",
    root / "lib_src/elements/publication/PhysicalStartup.cpp",
    root / "lib_src/elements/publication/PhysicalTransaction.cpp",
    root / "lib_src/elements/publication/ShellPhysicalScratchParticipation.cpp",
    root / "lib_src/collision/nodal_wall_mapped/Initialize.cpp",
):
    subprocess.run([*common, str(source)], check=True)
for source in (
    Path(__file__).resolve().parent / "OwnerStartup.cu",
    Path(__file__).resolve().parent / "ParticipationTest.cu",
    Path(__file__).resolve().parent.parent /
        "physical_mesh_wall" / "ParticipationTest.cu",
):
    subprocess.run([*common, "-x", "c++", str(source)], check=True)
print("fixed scratch participation public/implementation syntax: PASS")
