#!/usr/bin/env python3
"""One-CPU host shape check; expands nested CUDA wrappers but never invokes NVCC."""
from pathlib import Path
import argparse
import importlib.util
import json
import re
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
PREVIOUS = HERE.parent / 'solid_candidate_validation'
RESIDENT = ROOT / 'lib_src/elements/solids/resident'


def load_generated(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generated()


def expand_wrappers(value, generated):
    replacements = {
        '#include "lib_utest/qualification/solid_candidate_validation/ValidationTest.cu"':
            (PREVIOUS / 'ValidationTest.cu').read_text(),
        '#include "lib_utest/qualification/solid_candidate_validation/CurrentCandidate.cu"':
            (PREVIOUS / 'CurrentCandidate.cu').read_text(),
        '#include "lib_src/elements/solids/resident/Candidate.cu"':
            (RESIDENT / 'Candidate.cu').read_text(),
        '#include <FrozenCandidate.cu>': generated['FrozenCandidate.cu'],
    }
    for _ in range(len(replacements) + 1):
        changed = False
        for include, replacement in replacements.items():
            if include in value:
                value = value.replace(include, replacement)
                changed = True
        if not changed:
            break
    for include in replacements:
        assert include not in value, 'unexpanded nested wrapper: ' + include
    return value


def shaped(value):
    had_kernel = '__global__' in value
    value, launches = re.subn(r'<<<.*?>>>', '', value, flags=re.S)
    value = value.replace('__global__', '')
    if had_kernel:
        value = ('struct SyntaxIndex { unsigned x = 1; };\n'
                 'static SyntaxIndex blockIdx, blockDim, threadIdx, gridDim;\n') + value
    return value, launches


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('shape output must be create-only: ' + str(args.output))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.mkdir()

    generated = load_generated('solid_candidate_oracle', PREVIOUS / 'generate_oracle.py')
    generated.update(load_generated('solid_operand_oracle', HERE / 'generate_oracle.py'))
    for name, value in generated.items():
        (args.output / name).write_text(value)

    units = [
        HERE / 'CudaTest.cu',
        HERE / 'CurrentCandidate.cu',
        RESIDENT / 'Candidate.cu',
        RESIDENT / 'ResultValidation.cu',
        PREVIOUS / 'CandidateTest.cu',
        PREVIOUS / 'ValidationTest.cu',
        PREVIOUS / 'CurrentCandidate.cu',
        PREVIOUS / 'FrozenCandidate.cu',
    ]
    records = []
    for source in units:
        value = expand_wrappers(source.read_text(), generated)
        value, launches = shaped(value)
        relative = source.relative_to(ROOT)
        output = args.output / ('__'.join(relative.parts) + '.cpp')
        output.write_text(value)
        command = [
            'g++', '-std=c++17', '-DNDEBUG', '-fsyntax-only',
            '-fno-fast-math', '-ffp-contract=off',
            '-I' + str(ROOT), '-I' + str(args.output),
            '-I' + str(source.parent), '-I' + str(PREVIOUS), '-I' + str(RESIDENT),
            '-I/usr/local/cuda/include', '-I/usr/include/eigen3', str(output),
        ]
        subprocess.run(command, check=True)
        record = {
            'source': str(relative),
            'shaped_source': output.name,
            'removed_launches': launches,
            'expanded_nested_wrappers': source in {
                HERE / 'CudaTest.cu', HERE / 'CurrentCandidate.cu',
                PREVIOUS / 'CurrentCandidate.cu', PREVIOUS / 'FrozenCandidate.cu',
            },
        }
        records.append(record)
        print('PASS host shape', relative, flush=True)
    summary = {
        'status': 'passed',
        'units': records,
        'unit_count': len(records),
        'compiler': 'g++',
        'actual_cuda_compilation': False,
        'numerical_execution': False,
    }
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, sort_keys=True))


if __name__ == '__main__':
    main()
