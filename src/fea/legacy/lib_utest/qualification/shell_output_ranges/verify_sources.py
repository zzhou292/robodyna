#!/usr/bin/env python3
"""Exact wrapper removal proves the complete old scans and other checks remain."""
from pathlib import Path
import hashlib,json,runpy
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
BASELINE_HASH="a3ca98f8050a080af499aeab517db83e8ef4056b8c5e97c27497d3a1131e03cb"
MANIFEST_HASH="7792afa5319583621861d1917dc575453104fc68e853e0af07ebcf66c2e5511d"
def once(text,old,new):
    assert text.count(old)==1,(old,text.count(old))
    return text.replace(old,new)
def unwrap(text,marker):
    assert text.count(marker)==1,(marker,text.count(marker))
    start=text.index(marker);begin=text.index('{',start);depth=0
    for end in range(begin,len(text)):
        depth+=(text[end]=='{')-(text[end]=='}')
        if depth==0:break
    lines=text[begin+1:end].splitlines()
    while lines and not lines[0].strip():lines.pop(0)
    while lines and not lines[-1].strip():lines.pop()
    assert all(line.startswith('  ') for line in lines)
    return text[:start]+'\n'.join(line[2:] for line in lines)+text[end+1:]
def verify():
    raw=(HERE/'baseline.json').read_bytes();assert hashlib.sha256(raw).hexdigest()==BASELINE_HASH
    baseline=json.loads(raw)
    assert baseline['commit']=='341d60f090f2a1c7a1282fdd5d043b2387f98580'
    for row in baseline['files']:
        raw=(HERE/row['fixture']).read_bytes()
        assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256']
    raw=(HERE/'source-manifest.json').read_bytes();assert hashlib.sha256(raw).hexdigest()==MANIFEST_HASH
    manifest=json.loads(raw)
    for row in manifest['unchanged']+[manifest['helper']]:
        raw=(ROOT/row['path']).read_bytes()
        assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256'],row['path']
    generated=runpy.run_path(str(HERE/'generate_oracle.py'))['oracle_files']()
    for name,text in generated.items():assert (HERE/'oracle'/name).read_text()==text,name
    current=(ROOT/'lib_src/elements/ShellFormulationOutputRanges.h').read_text()
    current=once(current,'#include "ShellSourceRangeFilter.h"\n','')
    current=once(current,'  // Each family owns one contiguous Parent array of reference/node subobjects.\n','')
    current=once(current,'  // Catalog declarations are strided within owned Parent records; failure rows\n'
        '  // are contiguous. Keep the exact interleaved scan if either envelope overlaps.\n','')
    for family in ('qbat','qeph','t3'):
        current=unwrap(current,'  if (binding.'+family+'_count() && !shell_source_range_detail::DisjointEnvelope')
    current=unwrap(current,'  if (catalog.parent_count() &&\n')
    assert current==(HERE/'frozen/ShellFormulationOutputRanges.h').read_text()
    current=(ROOT/'lib_src/elements/ShellExecutionOutputRanges.cpp').read_text()
    current=once(current,'#include "ShellSourceRangeFilter.h"\n','')
    current=unwrap(current,'  if (catalog.parent_count() && !shell_source_range_detail::DisjointEnvelope')
    assert current==(HERE/'frozen/ShellExecutionOutputRanges.cpp').read_text()
    assert (ROOT/'lib_src/elements/ShellExecutionOutputRanges.h').read_bytes()==(HERE/'frozen/ShellExecutionOutputRanges.h').read_bytes()
    return {'status':'passed','complete_formulation_and_execution_reversals':True,
        'frozen_predicates_are_full_baseline_functions':True,'storage_abi_accessors_forecasts_and_numeric_sources_unchanged':True,
        'numerical_execution':False}
if __name__=='__main__':print(json.dumps(verify(),sort_keys=True))
