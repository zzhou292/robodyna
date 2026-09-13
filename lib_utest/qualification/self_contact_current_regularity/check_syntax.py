#!/usr/bin/env python3
"""Compile the public host API shape without linking or executing it."""

from pathlib import Path
import argparse
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("--compiler", required=True)
parser.add_argument("--output", required=True)
args = parser.parse_args()

here = Path(__file__).resolve().parent
root = here.parents[2]
output = Path(args.output)
output.mkdir(parents=True, exist_ok=True)
source = output / "public_api.cpp"
source.write_text(r'''
#include "lib_src/collision/SelfContactCurrentRegularity.h"
#include <type_traits>
using namespace tlfea::contact;
static_assert(std::is_copy_constructible_v<SelfContactCurrentRegularityReceipt>);
static_assert(!std::is_copy_constructible_v<SelfContactCurrentRegularity>);
static_assert(std::is_move_constructible_v<SelfContactCurrentRegularity>);
void Shape(SelfContactCurrentRegularity& owner,
           const SelfContactActiveUseBinding& binding,
           VectorView positions, SelfContactActivityView activity,
           const SelfContactPairClassification& pair) {
  auto plan = SelfContactCurrentRegularity::Preflight(binding);
  (void)plan;
  SelfContactCurrentRegularityReceipt receipt;
  (void)owner.Certify(positions, activity, &receipt);
  SelfContactPairClassification output;
  (void)owner.ExcludeCertifiedOwnParent(pair, receipt, &output);
  (void)owner.results();
}
''')
subprocess.run([
    args.compiler, "-std=c++17", "-fno-fast-math", "-ffp-contract=off",
    "-I", str(root), "-fsyntax-only", str(source),
], check=True)
