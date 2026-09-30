#!/usr/bin/env python3
"""Check literal factory composition and both callers' original read boundaries."""
from pathlib import Path
import hashlib,json,re
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[4]
BASELINE_SHA256="86541e9013856c3cee96c923f275dde5ae0cc9e31fa88be9af8b75230532e6ce"
def once(text,old,new):
    assert text.count(old)==1,(old,text.count(old))
    return text.replace(old,new)
def baseline():
    raw=(HERE/'baseline.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest()==BASELINE_SHA256
    manifest=json.loads(raw)
    assert manifest['commit']=='2776bee3e4e7f5662c5f00ea522d70b33f9998b8'
    originals={}
    for row in manifest['files']:
        raw=(HERE/row['fixture']).read_bytes()
        assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256'],row['path']
        originals[Path(row['path']).name]=raw.decode()
    for row in manifest['unchanged']:
        raw=(ROOT/row['path']).read_bytes()
        assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256'],row['path']
    return originals
PRODUCTION_SEARCH='    const auto geometry=tied::TiedShellSearchGeometry::Prepare(\n        tied::TiedShellPacking::Prepare(source.tied),source.member);\n    const auto finalized=vehicle_startup::TiedSearchFinalized::Prepare(\n        vehicle_startup::TiedSearchAssessment::Prepare(geometry));'
PRODUCTION_POST='    return vehicle_startup::TiedSearchPostKinChk::Prepare(\n        vehicle_startup::TiedSearchClassification::Prepare(finalized,context));'
def transform_original_yaris(old):
    text=once(old,'#include "case/vehicle_startup/physical_attachments/VehiclePhysicalAttachments.h"',
        '#include "case/vehicle_startup/physical_attachments/VehiclePhysicalAttachments.h"\n'
        '#include "case/vehicle_startup/physical_attachments/OriginalTiedPost.h"')
    text=once(text,PRODUCTION_SEARCH,
        '    const auto finalized=vehicle_startup::physical_attachments::FinalizeOriginalTiedSearch(\n'
        '        source.tied,source.member);')
    return once(text,PRODUCTION_POST,
        '    return vehicle_startup::physical_attachments::PrepareOriginalTiedPost(finalized,context);')
def check_factories(originals):
    old=originals['OriginalYaris.cpp']
    assert old.count(PRODUCTION_SEARCH)==1 and old.count(PRODUCTION_POST)==1
    search=PRODUCTION_SEARCH.replace('source.tied','declaration').replace('source.member','member')
    search=search.replace('const auto finalized=','return ').replace('vehicle_startup::','')
    post=PRODUCTION_POST.replace('vehicle_startup::','')
    expected=('#include "OriginalTiedPost.h"\n'
        'namespace crash::cases::vehicle_startup::physical_attachments {\n'
        'TiedSearchFinalized FinalizeOriginalTiedSearch(\n'
        ' const modelio::tied_shell::TiedShellDeclaration& declaration, const std::string& member) {\n'+search+'\n}\n'
        'TiedSearchPostKinChk PrepareOriginalTiedPost(\n'
        ' const TiedSearchFinalized& finalized, const modelio::tied_shell::TiedClassificationContext& context) {\n'+post+'\n}\n'
        '} // namespace crash::cases::vehicle_startup::physical_attachments\n')
    compact=lambda text:re.sub(r'\s+','',text)
    actual=(ROOT/'case/vehicle_startup/physical_attachments/OriginalTiedPost.cpp').read_text()
    assert compact(actual)==compact(expected),'only literal existing factory expressions may move'
def restore_original_yaris(current):
    originals=baseline();check_factories(originals)
    assert current==transform_original_yaris(originals['OriginalYaris.cpp'])
    return originals['OriginalYaris.cpp']
def verify():
    originals=baseline();check_factories(originals)
    restore_original_yaris((ROOT/'case/vehicle_run/source/OriginalYaris.cpp').read_text())
    old=originals['PreparePost.h']
    current=once(old,'#include "../VehiclePhysicalAttachments.h"',
        '#include "../VehiclePhysicalAttachments.h"\n#include "../OriginalTiedPost.h"')
    search='        const auto geometry = tied::TiedShellSearchGeometry::Prepare(\n            tied::TiedShellPacking::Prepare(source.tied_source()), member);\n        const auto finalized = TiedSearchFinalized::Prepare(TiedSearchAssessment::Prepare(geometry));'
    current=once(current,search,'        const auto finalized = FinalizeOriginalTiedSearch(source.tied_source(), member);')
    current=once(current,'return TiedSearchPostKinChk::Prepare(TiedSearchClassification::Prepare(finalized, context));',
        'return PrepareOriginalTiedPost(finalized, context);')
    assert (ROOT/'case/vehicle_startup/physical_attachments/tests/PreparePost.h').read_text()==current
    old=originals['TiedSearchPostKinChk.cmake']
    current=once(old,'add_library(robo_dyna_tied_search_post_kinchk STATIC "${CMAKE_CURRENT_LIST_DIR}/TiedSearchPostKinChk.cpp")',
        'add_library(robo_dyna_tied_search_post_kinchk STATIC "${CMAKE_CURRENT_LIST_DIR}/TiedSearchPostKinChk.cpp"\n'
        '  "${CMAKE_CURRENT_LIST_DIR}/physical_attachments/OriginalTiedPost.cpp")')
    assert (ROOT/'case/vehicle_startup/TiedSearchPostKinChk.cmake').read_text()==current
    assert (ROOT/'case/vehicle_startup/physical_attachments/VehiclePhysicalAttachments.cmake').read_text()==originals['VehiclePhysicalAttachments.cmake']
    return {'status':'passed','both_complete_callers_reversed':True,
        'original_auxiliary_context_and_byte_read_positions_preserved':True,
        'literal_factory_bodies_and_default_limits':True,'lower_producers_and_forecasts_unchanged':True,
        'existing_product_target_link_dependencies_unchanged':True,'numerical_execution':False}
if __name__=='__main__':print(json.dumps(verify(),sort_keys=True))
