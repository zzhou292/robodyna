#pragma once
#include "../TopologyAssessmentInternal.h"
#include "modelio/tied_shell/tests/TinyFixture.h"
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::test {
namespace d=detail;
struct Fixture {
  modelio::tied_shell::test::TinyFixture original{false,true};
  source::CanonicalData canonical=original.canonical;
  selection::Data selected;
  Fixture() {
    canonical.inputs.units={"t","mm","s",1000,.001,1};
    canonical.inputs.canonical_manifest={"manifest.json",output::Sha256(canonical.canonical_bytes),canonical.canonical_bytes.size()};
    canonical.inputs.scope_report={"scope.json",output::Sha256(canonical.scope_bytes),canonical.scope_bytes.size()};
    Set<double>("node_positions",{-0.,0,0,.001,0,0,.001,.001,0,0,.001,0,0,.002,0,
        .003,0,0,.004,0,0,.005,0,0,.006,0,0,.007,0,0,.008,0,0,0,0,0},3);
    Set<std::uint32_t>("shells_source_lines",{101,102},1);
    selected.selected_part_ids={100,101,200,201};selected.parts={
      {100,2100,1100,true,true,false,1,0,0},{101,2101,1101,true,true,false,1,0,0},
      {200,2200,1200,false,false,false,0,0,1},{201,2201,1201,false,false,false,0,1,0}};
    selected.counts={4,2,2,0,2,2,2,0,1,1};selected.startup_budget_bytes=1u<<20;
    selected.auxiliary_sha256=std::string(64,'a');selected.combine_sha256=std::string(64,'b');
  }
  template<class T> void Set(const char* name,const std::vector<T>& values,std::size_t columns) {
    auto it=std::find_if(canonical.arrays.begin(),canonical.arrays.end(),[&](const auto& a){return a.name==name;});
    if(it==canonical.arrays.end())it=std::find_if(canonical.arrays.begin(),canonical.arrays.end(),[](const auto& a){return a.name.empty();});
    if(it==canonical.arrays.end())throw std::runtime_error("No canonical fixture array slot");
    output::arrays::Layout shape{output::arrays::detail::Type<T>::value,values.size()/columns,columns,{}};
    it->name=name;it->bytes=output::arrays::Encode<T>(shape,values.data(),values.size());
    it->descriptor={std::string(name)+".bin",shape,it->bytes.size(),output::Sha256(it->bytes)};
  }
  template<class T> std::vector<T> Get(const char* name) const {
    const auto& a=source::FindArray(canonical,name);return output::arrays::Decode<T>(a.descriptor,a.bytes);
  }
  d::Inputs Inputs() const {Counts counts;return d::PrepareInputs(canonical,selected,{}, {},counts);}
};
struct Built {
  d::Inputs input;
  tl::util::HostArena arena,scratch;
  s::Snapshot view;
  explicit Built(const Fixture& f):input(f.Inputs()) {
    const auto plan=s::Preflight(input.View());
    if(plan.status!=s::Status::Ok||!arena.Initialize(plan.output_bytes)||!scratch.Initialize(plan.scratch_bytes))throw std::runtime_error("Tiny topology arena");
    if(s::BuildStarter(input.View(),{},arena,scratch,&view).status!=s::Status::Ok)throw std::runtime_error("Tiny general startup");
  }
};
struct Copied {
  s::Snapshot source;
  std::vector<s::Main> mains;
  std::vector<std::uint32_t> expanded,partners,offsets,incidence;
  std::vector<tlfea::contact::radioss_type25::StoredNormal> normals;
  std::vector<s::NormalReference> references;
  explicit Copied(const s::Snapshot& v):source(v),mains(v.mains,v.mains+v.main_count),
      expanded(v.expanded_to_primary,v.expanded_to_primary+v.main_count),partners(v.primary_to_partner,v.primary_to_partner+v.primary_count),
      offsets(v.normal_offsets,v.normal_offsets+v.starter.reference_count+1),incidence(v.normal_mains,v.normal_mains+v.normal_incidence_count),
      normals(v.starter.face_normals,v.starter.face_normals+4*v.main_count),references(v.starter.references,v.starter.references+v.starter.reference_count){}
  s::Snapshot View() const {
    auto v=source;v.mains=mains.data();v.expanded_to_primary=expanded.data();v.primary_to_partner=partners.data();
    v.normal_offsets=offsets.data();v.normal_mains=incidence.data();v.starter={normals.data(),references.data(),references.size()};return v;
  }
};
} // namespace crash::cases::vehicle_self_contact::native::test
