#!/usr/bin/env python3
"""Host/public and CUDA-shaped syntax only; no NVCC or GPU execution."""

from pathlib import Path
import argparse
import re
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]

parser = argparse.ArgumentParser()
parser.add_argument("--compiler", required=True)
parser.add_argument("--output", required=True, type=Path)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)

flags = [
    args.compiler, "-std=c++17", "-fno-fast-math", "-ffp-contract=off",
    "-I" + str(ROOT), "-I/usr/local/cuda/include",
    "-I/usr/include/eigen3", "-fsyntax-only",
]

consumer = args.output / "PublicConsumer.cpp"
consumer.write_text(r'''
#include "lib_src/collision/SelfContactForceAssembly.h"
#include <type_traits>
using namespace tlfea::contact;
static_assert(!std::is_copy_constructible_v<SelfContactForceAssembly>);
static_assert(std::is_copy_constructible_v<SelfContactForceAssemblyReceipt>);
void Shape(SelfContactForceAssembly& force,
           const SelfContactActiveUseBinding& binding,
           tl::fea::FENodalState& owner,
           const tl::fea::NodalTrialToken& token,
           const tl::fea::NodalAssemblyView& view,
           SelfContactForceEventView events) {
  SelfContactForceConfig config;
  auto plan = SelfContactForceAssembly::Forecast(config, binding);
  (void)plan;
  SelfContactForceAssemblyReceipt receipt;
  (void)force.Initialize(config, binding, owner);
  (void)force.AssembleAccepted(owner, token, view, events, &receipt);
  force.DiscardTrial();
  (void)force.allocations();
}
''')

for source in [
    consumer,
    ROOT / "lib_src/collision/self_contact_force/Values.cpp",
    ROOT / "lib_src/collision/self_contact_force/Source.cpp",
    ROOT / "lib_src/collision/self_contact_force/Layout.cpp",
    ROOT / "lib_src/collision/self_contact_force/Initialize.cpp",
]:
    subprocess.run(flags + [str(source)], check=True)
    print("PASS host shape", source.name, flush=True)

prefix = r'''
struct SelfContactCudaShapeIndex { unsigned x = 0; };
static SelfContactCudaShapeIndex blockIdx, blockDim, threadIdx, gridDim;
'''
for source in [
    ROOT / "lib_src/collision/self_contact_force/Operations.cu",
    HERE / "CudaTest.cu",
]:
    preprocessed = args.output / (source.stem + ".ii")
    with preprocessed.open("w") as output:
        subprocess.run([
            args.compiler, "-std=c++17", "-E", "-x", "c++", "-DNDEBUG",
            "-D__global__=", "-D__device__=", "-D__shared__=",
            "-I" + str(ROOT), "-I/usr/local/cuda/include",
            "-I/usr/include/eigen3", str(source),
        ], stdout=output, check=True)
    text = re.sub(r"<<<.*?>>>", "", preprocessed.read_text(), flags=re.S)
    shaped = args.output / (source.stem + ".cpp")
    shaped.write_text(prefix + text)
    subprocess.run(flags + [str(shaped)], check=True)
    print("PASS CUDA-shaped syntax", source.name, flush=True)
