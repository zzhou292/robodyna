"""Extract complete original validation loops; decorate only source/transport seams."""
from pathlib import Path
import re
HERE=Path(__file__).resolve().parent

def body(text,name):
    start=text.index('{',text.index(name+'('));end=start+1;depth=1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[start+1:end-1]

def tail(file,name,marker):
    value=body((HERE/'reference'/file).read_text(),name)
    return value[value.index(marker):]

def words(text,**mapping):
    for old,new in mapping.items():text=re.sub(r'(?<![.\w])'+old+r'\b',new,text)
    return text

def generate():
    point=tail('ShellOnePointStorage.cpp.txt','OnePointHostStorage::Read','  for (std::size_t e')
    point=point.replace('catalog.Law(ShellBindingFamily::T3, e, &law)','ReadLaw(f,e,&law)').replace('catalog.Parameters(ShellBindingFamily::T3, e, &parameters)','ReadParameters(f,e,&parameters)').replace('staging_[e]','f.points[e]')
    point=words(point,count='f.size()',time='f.time')
    mixed=tail('ShellMixedSectionReadback.cpp.txt','MixedHostStorage::Read','  // Complete finite')
    mixed=mixed.replace('catalog.Law(family_,e,&law)','ReadLaw(f,e,&law)').replace('catalog.execution_sections()','f.execution').replace('elastic_[e]','f.elastic[e]').replace('plastic_[e]','f.plastic[e]').replace('output_[e]','f.sections[e]')
    mixed=words(mixed,count='f.size()',family_='ShellBindingFamily::T3')
    mixed='  const auto* one_point=f.has_point?f.points.data():nullptr;\n'+mixed
    failure=body((HERE/'reference/ShellFailureStorage.cpp.txt').read_text(),'FailureHostStorage::Read')
    failure=failure[failure.rfind('  for (std::size_t e'):]
    failure=failure.replace('const auto* source = binding_.parent(family_, e);','const FailureSource declared{f.policies[e]}; const auto* source=e==f.missing_failure?nullptr:&declared;').replace('staging_[e]','f.failure[e]').replace('sections[e]','f.sections[e]')
    failure=words(failure,count='f.size()',time='f.time')
    force=tail('Readback.cpp.txt','T3Batch::Impl::ValidateMappedResults','  for (std::size_t parent')
    force=force.replace('physical->catalog()->Law(ShellBindingFamily::T3,parent,&law)','ReadLaw(f,parent,&law)').replace('physical->shells()->t3_reference(parent)','f.elements[parent].reference').replace('staging[parent]','f.forces[parent]').replace('config.element_count','f.size()')
    force=words(force,time='f.force_time',epoch='f.force_epoch')
    roles=tail('Readback.cpp.txt','T3Batch::Impl::ValidateMappedSections','  for (std::size_t parent')
    roles=roles.replace('physical->catalog()->Law(ShellBindingFamily::T3,parent,&law)','ReadLaw(f,parent,&law)').replace('sections[parent]','f.sections[parent]').replace('failure[parent]','f.failure[parent]').replace('staging[parent]','f.forces[parent]').replace('config.element_count','f.size()')
    identity=tail('T3BatchOnePointReadback.cpp.txt','ReadOnePoint','  for (std::size_t e')
    identity=identity.replace('sections[e]','f.sections[e]').replace('state.staging[e]','f.forces[e]').replace('state.joined_binding->t3_reference(e)','f.elements[e].reference').replace('catalog->Parameters(ShellBindingFamily::T3, e, &parameters)','ReadParameters(f,e,&parameters)').replace('state.config.element_count','f.size()')
    identity=words(identity,time='f.time',epoch='f.epoch')
    out="""// Generated from complete pinned pre-change loops; no predicate substitution.
#pragma once
#include "Fixture.h"
namespace t3_compact_test::serial {
using namespace tl::fea;using namespace tl::fea::t3;
using namespace tl::fea::shell_batch_plasticity_detail;
struct FailureSource { ShellFailurePolicy policy; };
inline bool ReadLaw(const Fixture& f,std::size_t p,ShellSectionLaw* value) {
  if(p==f.missing_law)return false;*value=f.laws[p];return true;
}
inline bool ReadParameters(const Fixture& f,std::size_t p,sections::PointParameters* value) {
  if(p==f.missing_parameter)return false;*value=f.parameters[p];return true;
}
"""
    for name,value in [('Points',point),('Mixed',mixed),('Failure',failure),('Force',force),('Roles',roles),('PointIdentity',identity)]:
        report='SetupReport' if name in ('Points','Mixed','Failure') else 'BatchReport'
        out+=f'inline {report} {name}(Fixture& f) {{{value}}}\n'
    return out+'} // namespace t3_compact_test::serial\n'

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);args=parser.parse_args()
    with args.output.open('x') as stream:stream.write(generate())
