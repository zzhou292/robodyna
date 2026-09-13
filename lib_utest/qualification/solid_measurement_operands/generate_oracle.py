#!/usr/bin/env python3
"""Adapt the complete pre-operand arena by includes and namespace only."""
from pathlib import Path
import argparse
HERE = Path(__file__).resolve().parent
PREFIX = 'lib_src/elements/solids/resident/'
def generated():
    header = (HERE / 'frozen/native_base/Arena.h.txt').read_text()
    for name in ['Batch.h', 'ExtendedScratch.h', 'AssemblyTypes.h']:
        header = header.replace(f'#include "{name}"', f'#include "{PREFIX}{name}"')
    header = header.replace('#include "../../ShellPhysicalOwner.h"',
                            '#include "lib_src/elements/ShellPhysicalOwner.h"')
    header = header.replace('namespace tl::fea::solids::batch_detail {',
                            'namespace tl::fea::solids::batch_detail::operand_baseline {')
    source = (HERE / 'frozen/native_base/Arena.cpp.txt').read_text()
    source = source.replace('#include "Arena.h"', '#include "OperandBaselineArena.h"')
    source = source.replace('namespace tl::fea::solids::batch_detail {',
                            'namespace tl::fea::solids::batch_detail::operand_baseline {')
    return {'OperandBaselineArena.h': header, 'OperandBaselineArena.cpp': source}
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for name, value in generated().items():
        (args.output / name).write_text(value)
