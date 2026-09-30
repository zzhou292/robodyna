#!/usr/bin/env python3
"""C++ shape only: preprocess owning includes, erase CUDA launch syntax.

This never invokes NVCC, generates device code, or executes a GPU kernel.
"""
from pathlib import Path
import argparse
import re
import subprocess

here = Path(__file__).resolve().parent
root = here.parents[2]
p = argparse.ArgumentParser()
p.add_argument('--output', type=Path, required=True)
args = p.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
prefix = '''
struct SyntaxIndex { unsigned x = 0; };
static SyntaxIndex blockIdx, blockDim, threadIdx, gridDim;
inline void __syncthreads() {}
inline unsigned __ballot_sync(unsigned, bool) { return 0; }
template<class T, class U> T atomicMin(T*, U) { return {}; }
'''
for source in [root/'lib_src/collision/nodal_wall_mapped/Operations.cu', here/'CudaTest.cu']:
    preprocessed = args.output/(source.stem+'.ii')
    with preprocessed.open('w') as out:
        subprocess.run(['g++', '-std=c++17', '-E', '-x', 'c++', '-DNDEBUG',
            '-D__global__=', '-D__device__=', '-D__shared__=',
            '-I'+str(root), '-I/usr/local/cuda/include', '-I/usr/include/eigen3',
            str(source)], stdout=out, check=True)
    text = re.sub(r'<<<.*?>>>', '', preprocessed.read_text(), flags=re.S)
    shaped = args.output/(source.stem+'.cpp')
    shaped.write_text(prefix+text)
    subprocess.run(['g++', '-std=c++17', '-fsyntax-only', str(shaped)], check=True)
    print('PASS host shape', str(source.relative_to(root)), flush=True)
