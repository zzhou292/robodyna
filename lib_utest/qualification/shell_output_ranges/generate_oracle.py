#!/usr/bin/env python3
"""Namespace/include adaptation only, using complete frozen341 predicates."""
from pathlib import Path
import argparse,hashlib,json
HERE=Path(__file__).resolve().parent
def oracle_files():
    values=json.loads((HERE/'baseline.json').read_text())
    assert values['commit']=='341d60f090f2a1c7a1282fdd5d043b2387f98580'
    output={}
    for row in values['files']:
        raw=(HERE/row['fixture']).read_bytes()
        assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256']
        text=raw.decode()
        text=text.replace('namespace tl::fea::shell_formulation_detail {','namespace tl::fea::shell_formulation_detail::frozen341 {')
        text=text.replace('namespace tl::fea::shell_execution_detail {','namespace tl::fea::shell_execution_detail::frozen341 {')
        for header in ('ShellFormulationScope.h','ShellCatalogCurveView.h'):
            text=text.replace('#include "'+header+'"','#include "lib_src/elements/'+header+'"')
        text=text.replace('#include "../assembly/','#include "lib_src/assembly/')
        text=text.replace('#include "../solvers/','#include "lib_src/solvers/')
        text=text.replace('#include "ShellExecutionOutputRanges.h"','#include "FrozenExecution.h"')
        name={'ShellFormulationOutputRanges.h':'FrozenFormulation.h','ShellExecutionOutputRanges.h':'FrozenExecution.h','ShellExecutionOutputRanges.cpp':'FrozenExecution.cpp'}[Path(row['fixture']).name]
        output[name]=text
    return output

def generate(output):
    output.mkdir(parents=True,exist_ok=True)
    for name,text in oracle_files().items():(output/name).write_text(text)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    generate(p.parse_args().output)
