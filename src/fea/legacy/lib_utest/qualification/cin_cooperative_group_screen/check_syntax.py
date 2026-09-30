#!/usr/bin/env python3
"""Host shape only. No CUDA compilation/execution; launch syntax is erased."""
from pathlib import Path
import argparse
import re
import subprocess
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
p = argparse.ArgumentParser()
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
for name in ['lib_src/solvers/cin_advance/Screen.cu',
             str((HERE/'FrozenScreen.cu').relative_to(ROOT)),
             str((HERE/'CudaTest.cu').relative_to(ROOT))]:
    path = ROOT/name
    text = path.read_text()
    if path.name == 'Screen.cu':
        # Expand only our private device helper, preserving its actual includes.
        text = text.replace('#include "GroupScreen.cuh"',
            (path.parent/'GroupScreen.cuh').read_text().replace('#pragma once', ''))
    text = re.sub(r'<<<.*?>>>', '', text, flags=re.S)
    text = text.replace('__global__', '').replace('__device__', '').replace('__shared__', '')
    text = ('struct SyntaxIndex { unsigned x = 1; };\n'
        'static SyntaxIndex blockIdx, blockDim, threadIdx, gridDim;\n'
        'inline void __syncthreads() {}\n')+text
    output = a.output/(path.name+'.cpp')
    output.write_text(text)
    subprocess.run(['g++','-std=c++17','-DNDEBUG','-fsyntax-only',
        '-I'+str(ROOT),'-I'+str(path.parent),'-I'+str(ROOT/'lib_src/solvers'),
        '-I/usr/local/cuda/include','-I/usr/include/eigen3',str(output)], check=True)
    print('PASS host shape',name,flush=True)
