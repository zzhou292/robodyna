#!/usr/bin/env python3
"""Adapt complete dfa0a93 caller bodies by includes and namespace only."""
from pathlib import Path
import argparse

HERE = Path(__file__).resolve().parent
PREFIX = 'lib_src/elements/solids/resident/'


def generated():
    measure = (HERE / 'frozen/Measure.h.txt').read_text()
    for name in ['DeviceFamilies.h', 'ResultChecks.h']:
        measure = measure.replace(f'#include "{name}"', f'#include "{PREFIX}{name}"')
    measure = measure.replace('#include "../../ShellBatchFields.h"',
                              '#include "lib_src/elements/ShellBatchFields.h"')
    measure = measure.replace('namespace tl::fea::solids::batch_detail {',
                              'namespace tl::fea::solids::batch_detail::frozen {')
    candidate = (HERE / 'frozen/Candidate.cu.txt').read_text()
    candidate = candidate.replace('#include "Storage.h"', f'#include "{PREFIX}Storage.h"')
    candidate = candidate.replace('#include "Measure.h"', '#include "FrozenMeasure.h"')
    candidate = candidate.replace('namespace tl::fea::solids::batch_detail {',
                                  'namespace tl::fea::solids::batch_detail::frozen {')
    arena = (HERE / 'frozen/Arena.h.txt').read_text()
    for name in ['Batch.h', 'ExtendedScratch.h']:
        arena = arena.replace(f'#include "{name}"', f'#include "{PREFIX}{name}"')
    arena = arena.replace('#include "../../ShellPhysicalOwner.h"',
                          '#include "lib_src/elements/ShellPhysicalOwner.h"')
    arena = arena.replace('namespace tl::fea::solids::batch_detail {',
                          'namespace tl::fea::solids::batch_detail::baseline_layout {')
    layout = (HERE / 'frozen/Arena.cpp.txt').read_text()
    layout = layout.replace('#include "Arena.h"', '#include "FrozenArena.h"')
    layout = layout.replace('namespace tl::fea::solids::batch_detail {',
                            'namespace tl::fea::solids::batch_detail::baseline_layout {')
    return {'FrozenMeasure.h': measure, 'FrozenCandidate.cu': candidate,
            'FrozenArena.h': arena, 'FrozenArena.cpp': layout}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for name, value in generated().items():
        (args.output / name).write_text(value)
