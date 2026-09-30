#!/usr/bin/env python3
"""Compile public and implementation activity shapes without NVCC."""
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
    '#include "lib_src/collision/SelfContactPhysicalActivity.h"\n'
    '#include "lib_src/collision/self_contact_physical_activity/Selection.h"\n'
    "#include <type_traits>\n"
    "using namespace tlfea::contact;\n"
    "static_assert(!std::is_aggregate_v<SelfContactAcceptedActivityReceipt>);\n"
    "static_assert(!std::is_aggregate_v<SelfContactPreparedActivityReceipt>);\n"
    "static_assert(!std::is_copy_constructible_v<SelfContactPhysicalActivity>);\n"
    "int main(){ SelfContactAcceptedActivityReceipt a; "
    "SelfContactPreparedActivityReceipt p; "
    "tl::fea::ShellPlasticityParentInput row; "
    "const auto selection = self_contact_physical_activity::ValidateSelectionRows("
    "0, [&](std::size_t) noexcept -> const tl::fea::ShellPlasticityParentInput& "
    "{ return row; }, [](const auto&) noexcept { return false; }); "
    "return a.valid() || p.valid() || "
    "selection.status != SelfContactPhysicalActivityStatus::InvalidInput; }\n"
)

common = [
    args.compiler,
    "-std=c++17",
    "-fno-fast-math",
    "-ffp-contract=off",
    "-I", str(root),
    "-I", "/usr/local/cuda/include",
    "-I", "/usr/include/eigen3",
    "-fsyntax-only",
]
for source in (
    probe,
    root / "lib_src/collision/self_contact_physical_activity/Layout.cpp",
    root / "lib_src/collision/self_contact_physical_activity/Values.cpp",
    root / "lib_src/collision/self_contact_physical_activity/Activity.cpp",
    root / "lib_src/elements/publication/PhysicalReadback.cpp",
):
    subprocess.run([*common, str(source)], check=True)

print("self-contact physical activity syntax: PASS")
