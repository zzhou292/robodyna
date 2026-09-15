#!/usr/bin/env python3
"""Bounded C++ source-shape checks; no CUDA compiler or execution."""

from pathlib import Path
import argparse
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]

parser = argparse.ArgumentParser()
parser.add_argument("--compiler", required=True)
parser.add_argument("--output", required=True, type=Path)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)

consumer = args.output / "PublicConsumer.cpp"
consumer.write_text(
    """
#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"
#include <type_traits>
using namespace tlfea::contact;
static_assert(!std::is_copy_constructible_v<FixedTriangleFeatureDiscovery>);
static_assert(std::is_trivially_copyable_v<CurrentFixedTriangle>);
static_assert(std::is_trivially_copyable_v<FixedTriangleFeatureCandidate>);
static_assert(std::is_trivially_copyable_v<FixedTriangleIntersection>);
int main() {
  FixedTriangleFeatureDiscovery discovery;
  CurrentFixedTriangle triangles[2];
  FixedTriangleFeatureTaskMask mask;
  auto mask_status = BuildFixedTriangleFeatureTaskMask(
      triangles[0], triangles[1], &mask);
  auto plan = FixedTriangleFeatureDiscovery::Preflight();
  auto report = discovery.Discover(nullptr, 0, nullptr, 0);
  auto f = discovery.features();
  auto x = discovery.intersections();
  return int(mask_status) + int(plan.report.status) +
         int(report.feature_tasks) +
         int(f.complete) + int(x.complete);
}
"""
)

sources = [
    consumer,
    ROOT / "lib_src/collision/fixed_triangle_features/ExactPredicates.cpp",
    ROOT / "lib_src/collision/fixed_triangle_features/Geometry.cpp",
    ROOT / "lib_src/collision/fixed_triangle_features/Discovery.cpp",
]
for source in sources:
    command = [
        args.compiler,
        "-std=c++17",
        "-fno-fast-math",
        "-ffp-contract=off",
        "-I" + str(ROOT),
        "-fsyntax-only",
        str(source),
    ]
    subprocess.run(command, check=True)
    print("PASS C++ source shape", source.relative_to(ROOT)
          if source.is_relative_to(ROOT) else source.name, flush=True)

print("PASS: 4 C++ source-shaped units; no NVCC, CUDA, GPU, or numerical claim")
