#!/usr/bin/env python3
"""Host shape check only: kernel launch syntax is removed; never runs CUDA."""
from pathlib import Path
import argparse
import re
import subprocess
from generate_oracle import generated

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--output', required=True, type=Path)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
for name, value in generated().items():
    (args.output / name).write_text(value)
prefix = 'lib_src/elements/solids/resident/'
files = [prefix + name for name in ['Arena.cpp', 'Upload.cpp', 'Forecast.cpp',
                                   'Candidate.cu', 'ResultValidation.cu']]
files += [str((HERE / name).relative_to(ROOT)) for name in
          ['CandidateTest.cu', 'ValidationTest.cu', 'CurrentCandidate.cu', 'FrozenCandidate.cu']]
for name in files:
    source = ROOT / name
    value = source.read_text()
    value = value.replace('#include "lib_src/elements/solids/resident/Candidate.cu"',
                          (ROOT / prefix / 'Candidate.cu').read_text())
    value = value.replace('#include <FrozenCandidate.cu>', generated()['FrozenCandidate.cu'])
    if '__global__' in value:
        value = re.sub(r'<<<.*?>>>', '', value, flags=re.S)
        value = value.replace('__global__', '')
        value = ('struct SyntaxIndex { unsigned x = 1; };\n'
                 'static SyntaxIndex blockIdx, blockDim, threadIdx, gridDim;\n') + value
    output = args.output / (source.name + '.cpp')
    output.write_text(value)
    command = ['g++', '-std=c++17', '-DNDEBUG', '-fsyntax-only',
               '-I' + str(ROOT), '-I' + str(args.output), '-I' + str(source.parent),
               '-I' + str(ROOT / prefix),
               '-I/usr/local/cuda/include', '-I/usr/include/eigen3', str(output)]
    subprocess.run(command, check=True)
    print('PASS host shape', name, flush=True)
print('PASS: 9 host/shape units; no CUDA compiler or numerical execution')
