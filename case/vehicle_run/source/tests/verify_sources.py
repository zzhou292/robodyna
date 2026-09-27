#!/usr/bin/env python3
"""Authenticate the literal reader extraction and unchanged legacy construction."""
from pathlib import Path
import argparse
import hashlib
import json
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
BASELINE_SHA256="62537b9f1b021360b3b4830f7cd6628ec6ae3d91a8b034fb1f967f0ff2e6e2d9"
def once(text,old,new):
    assert text.count(old)==1,(old,text.count(old))
    return text.replace(old,new)
def verify(output=None):
    raw=(HERE/'baseline.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest()==BASELINE_SHA256
    baseline=json.loads(raw)
    assert baseline['commit']=='178366385e807bc268f485d6ce0962fa516f60a0'
    originals={}
    for row in baseline['files']:
        raw=(HERE/row['fixture']).read_bytes()
        assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256'],row['path']
        originals[Path(row['path']).name]=raw.decode()
    old=originals['OriginalSources.cpp']
    start=old.index('namespace records=')
    boundary=old.index('\n}\nstd::string ReadOriginal')
    reader_start=boundary+3
    constructor=old.index('OriginalSources::OriginalSources(')
    private=old[start:boundary]
    reader=old[reader_start:constructor]
    prefix='#include "OriginalSources.h"\nnamespace crash::cases::vehicle_run::detail {\n'
    source=ROOT/'case/vehicle_run/source'
    assert (source/'OriginalSources.cpp').read_text()==prefix+old[constructor:]
    expected_io=('#include "OriginalSourceIO.h"\n'
        '#include "modelio/vehicle_sections/VehicleSectionResolution.h"\n'
        'namespace crash::cases::vehicle_run::detail {\n'+private+'\n'+reader+
        '} // namespace crash::cases::vehicle_run::detail\n')
    assert (source/'OriginalSourceIO.cpp').read_text()==expected_io
    header=once(originals['OriginalSources.h'],'#include "OriginalYaris.h"','#include "OriginalSourceIO.h"')
    header=once(header,'std::string ReadOriginal(const std::filesystem::path&,std::size_t,const char* sha256);\n','')
    assert (source/'OriginalSources.h').read_text()==header
    assert (source/'OriginalYaris.cpp').read_text()=='#include "OriginalYaris.h"\n'+originals['OriginalYaris.cpp']
    cmake=once(originals['VehicleRun.cmake'],'add_library(robo_dyna_vehicle_run_original_source STATIC\n',
        'include("${CMAKE_CURRENT_LIST_DIR}/source/OriginalSourceIO.cmake")\n'
        'add_library(robo_dyna_vehicle_run_original_source STATIC\n')
    cmake=once(cmake,'target_link_libraries(robo_dyna_vehicle_run_original_source PUBLIC robo_dyna_vehicle_run)',
        'target_link_libraries(robo_dyna_vehicle_run_original_source PUBLIC robo_dyna_vehicle_run robo_dyna_original_source_io)')
    assert (ROOT/'case/vehicle_run/VehicleRun.cmake').read_text()==cmake
    if output:
        output.mkdir(parents=True,exist_ok=True)
        (output/'FrozenReadOriginal.h').write_text('#pragma once\n#include "output/ArtifactIO.h"\n'
            'namespace crash::cases::vehicle_run::detail::test::frozen {\n'+reader+'}\n')
    return {'status':'passed','three_complete_reader_bodies_identical':True,
        'constructor_and_field_order_unchanged':True,'legacy_case_body_unchanged':True,
        'numerical_execution':False}
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path)
    print(json.dumps(verify(parser.parse_args().output),sort_keys=True))
