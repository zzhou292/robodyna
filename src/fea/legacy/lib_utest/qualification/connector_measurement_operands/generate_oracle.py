#!/usr/bin/env python3
"""Use complete pinned serial measurement and original/current finalizer bodies."""
from pathlib import Path
import argparse
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
def body(text,name):
    start=text.index('__global__ void '+name+'(')
    begin=text.index('{',start); depth=0
    for end in range(begin,len(text)):
        depth+=(text[end]=='{')-(text[end]=='}')
        if depth==0:return text[start:end+1].replace('__global__ void '+name+'(','TL_SURFACE_HD inline void Finalize(',1)
    raise ValueError('Missing finalizer body')
def generate(output):
    output.mkdir(parents=True,exist_ok=True)
    entries=[('Type13','type13','resident/Measure.h','resident/Candidate.cu','Finalize'),
             ('Type25','type25','Type25BatchMeasure.h','Type25BatchKernels.cu','FinalizeCandidate')]
    finalizers=['#pragma once']
    for label,family,measure,candidate,finalize in entries:
        text=(HERE/'frozen'/f'{label}Measure.h').read_text()
        relative='lib_src/elements/'+family+'/'
        text=text.replace('#include "Arena.h"',f'#include "{relative}resident/Arena.h"')
        text=text.replace('#include "Type25BatchArena.h"',f'#include "{relative}Type25BatchArena.h"')
        text=text.replace('#include "../../ShellBatchFields.h"','#include "lib_src/elements/ShellBatchFields.h"')
        text=text.replace('#include "../ShellBatchFields.h"','#include "lib_src/elements/ShellBatchFields.h"')
        text=text.replace(f'namespace tl::fea::{family}::batch_detail {{',f'namespace tl::fea::{family}::batch_detail::reference {{')
        (output/f'Frozen{label}Measure.h').write_text(text)
        for scope,source in [('reference',(HERE/'frozen'/f'{label}Candidate.cu').read_text()),
                             ('qualification_current',(ROOT/relative/candidate).read_text())]:
            caller=body(source,finalize)
            if scope=='reference':
                # DeviceModel belongs to the live namespace. Keep ADL from
                # adding the production Measure to the frozen overload set.
                assert caller.count('Measure(')==1
                caller=caller.replace('Measure(',f'::tl::fea::{family}::batch_detail::reference::Measure(',1)
            finalizers.append(f'namespace tl::fea::{family}::batch_detail::{scope} {{\n'+caller+'\n}')
    for label,family in [('Type13','type13'),('Type25','type25')]:
        prefix='lib_src/elements/'+family+'/'
        header=(HERE/'frozen'/f'{label}Arena.h').read_text()
        if label=='Type13':
            header=header.replace('#include "Batch.h"',f'#include "{prefix}resident/Batch.h"')
            header=header.replace('#include "../../../../lib_utils/BoundedArena.h"','#include "lib_utils/BoundedArena.h"')
        else:
            header=header.replace('#include "Type25Batch.h"',f'#include "{prefix}Type25Batch.h"')
            header=header.replace('#include "../../../lib_utils/BoundedArena.h"','#include "lib_utils/BoundedArena.h"')
        header=header.replace(f'namespace tl::fea::{family}::batch_detail {{',f'namespace tl::fea::{family}::batch_detail::baseline_layout {{')
        source=(HERE/'frozen'/f'{label}Arena.cpp').read_text()
        old='Arena.h' if label=='Type13' else 'Type25BatchArena.h'
        source=source.replace(f'#include "{old}"',f'#include "Frozen{label}Arena.h"')
        source=source.replace(f'namespace tl::fea::{family}::batch_detail {{',f'namespace tl::fea::{family}::batch_detail::baseline_layout {{')
        (output/f'Frozen{label}Arena.h').write_text(header)
        (output/f'Frozen{label}Arena.cpp').write_text(source)
    (output/'FinalizeBodies.h').write_text('\n'.join(finalizers)+'\n')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    generate(p.parse_args().output)
