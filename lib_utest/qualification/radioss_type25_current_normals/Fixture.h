// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include "../radioss_type25_fixed_main_startup/Fixture.h"
#include "../radioss_type25_fixed_main_startup/NativeOracle.h"
#include "lib_src/collision/RadiossType25NormalActivation.h"
#include "lib_src/collision/radioss_type25/selection/lifecycle/Admission.h"
#include <algorithm>
#include <memory>
#include <numeric>
namespace type25_current_normals_test {
namespace s=n::startup;namespace l=n::selection::lifecycle;
using Case=type25_startup_test::Case;
using type25_startup_test::Grid;
inline Case ExtraSecondary(Case value) {
  value.ids.push_back(value.ids.size()+1);
  value.positions.insert(value.positions.end(),{.5,.5,.2});return value;
}
inline Case Cube() {
  Case x;x.positions={0,0,0,1,0,0,1,1,0,0,1,0,0,0,1,1,0,1,1,1,1,0,1,1};
  for(unsigned i=0;i<8;++i)x.ids.push_back(i+1);
  const std::uint32_t faces[6][4]{{0,3,2,1},{4,5,6,7},{0,1,5,4},{1,2,6,5},{2,3,7,6},{3,0,4,7}};
  for(const auto& face:faces)x.Add(n::ShellLayout::Quad4,face[0],face[1],face[2],face[3]);
  return x;
}
template<class StartupBuilder=type25_startup_test::Built>
struct FixtureT {
  Case mesh;
  StartupBuilder built;
  type25_startup_test::NativeResult native_start;
  std::vector<double> positions,coefficients;
  std::vector<std::uint32_t> main_active,node_tag,free_ids;
  std::vector<n::StoredNormal> prior,native_prior;
  explicit FixtureT(Case source):mesh(ExtraSecondary(std::move(source))),built(mesh),
      native_start(type25_startup_test::Oracle(mesh.Input(),mesh.coefficients.data(),mesh.coefficients.size())),
      positions(mesh.positions),coefficients(mesh.coefficients),main_active(built.startup.main_count,1),node_tag(mesh.ids.size(),1),
      prior(built.startup.starter.face_normals,built.startup.starter.face_normals+4*built.startup.main_count),native_prior(native_start.starter_normals){RefreshFree();}
  c::Input Input(bool native_topology=false) const {
    c::Input in;in.profile=c::Profile::OrdinaryShellLocal;in.free_roster=n::normal_activation::FreeRosterPolicy::FreshComplete;
    const auto& t=built.startup;in.topology={t.mains,t.node_count,t.primary_count,t.main_count,t.starter.reference_count,
      {t.normal_offsets,t.starter.reference_count+1,t.normal_mains,t.normal_incidence_count}};
    if(native_topology)in.topology={native_start.mains.data(),mesh.ids.size(),mesh.primary.size(),native_start.mains.size(),native_start.starter_references.size(),
      {native_start.offsets.data(),native_start.offsets.size(),native_start.incidence.data(),native_start.incidence.size()}};
    in.positions={positions.data(),std::uint32_t(mesh.ids.size()),3,1};in.coordinates=mesh.units;in.units=mesh.scale;
    in.main_coefficients=coefficients.data();in.coefficient_count=coefficients.size();in.main_active=main_active.data();in.active_count=main_active.size();
    in.node_tag=node_tag.data();in.tag_count=node_tag.size();in.free_main_ids=free_ids.empty()?nullptr:free_ids.data();in.free_count=free_ids.size();
    const auto& old=native_topology?native_prior:prior;in.prior_normals=old.data();in.prior_count=old.size();return in;
  }
  void RefreshFree() {
    free_ids.clear();const auto& t=built.startup;
    // This is fixture input, independently rechecked by original FREE_BOUND.
    for(std::size_t m=0;m<t.main_count;++m)if(coefficients[m]>0)for(unsigned j=0;j<4;++j)
      if(!t.mains[m].neighbors[j]&&!(j==2&&t.mains[m].nodes[2]==t.mains[m].nodes[3])){free_ids.push_back(std::uint32_t(m+1));break;}
  }
  void AllActive(){std::fill(main_active.begin(),main_active.end(),1);std::fill(node_tag.begin(),node_tag.end(),1);}
  // Derive actual integer masks through the qualified Begin/OPTCD -> TAG leaf,
  // rather than assuming every current face is active. No retained classification
  // or normal arithmetic is used to manufacture expected normal values.
  void GeneratedMasks(const std::vector<std::uint32_t>& raw_mains={},int retained=0,
      const std::vector<std::uint32_t>& removed={}) {
    const auto& t=built.startup;std::vector<l::Node> nodes(mesh.ids.size());std::vector<l::Main> mains(t.main_count);
    std::vector<l::NormalReference> refs(t.starter.reference_count);std::vector<double> velocities(positions.size());
    for(std::size_t i=0;i<nodes.size();++i)nodes[i].source_id=mesh.ids[i];
    for(std::size_t i=0;i<mains.size();++i){const auto& a=t.mains[i];auto& b=mains[i];b.global_id=a.global_id;b.segment_type=a.segment_type;b.coefficient=coefficients[i];b.maximum_gap=1000;
      for(unsigned j=0;j<4;++j){b.nodes[j]=a.nodes[j];b.normal_reference[j]=a.normal_reference[j];b.neighbors[j]=a.neighbors[j];b.normal_slot[j]=prior[4*i+j];b.gap[j]=1000;}}
    l::Secondary secondary{std::uint32_t(nodes.size()-1),400,1000,0};n::NativeGeometryHistory history;
    history.secondary_source_id=nodes.back().source_id;history.generation=7;
    if(retained){history.row.irtlm[0]=mains[retained-1].global_id;history.row.irtlm[1]=1;history.row.irtlm[2]=retained;history.row.irtlm[3]=1;
      history.row.history.normal={.1,100,.2,200,.05};}
    std::vector<l::SpatialOccurrence> raw;std::vector<std::uint32_t> raw_order;
    for(auto main:raw_mains){raw.push_back({1,int(main)});raw_order.push_back(std::uint32_t(raw_order.size()));}
    const std::uint32_t raw_offsets[]{0,std::uint32_t(raw.size())},removed_offsets[]{0,std::uint32_t(removed.size())};
    l::Input in;in.profile.selection={1,5,1,false,false,false};in.profile.geometry={1,1,5,1,false,false,false};
    in.profile.coefficient={4,0};in.profile.maximum_coefficient=1e30;in.profile.neighbor_removal=2;in.profile.optcd_response_precision=0;
    in.step={0,.001};in.source={nodes.data(),nodes.size(),mains.data(),mains.size(),&secondary,1,refs.data(),refs.size(),
      {t.normal_offsets,refs.size()+1,t.normal_mains,t.normal_incidence_count},{removed_offsets,2,removed.empty()?nullptr:removed.data(),removed.size()},7};
    in.current.positions={positions.data(),std::uint32_t(nodes.size()),3,1};in.current.velocities={velocities.data(),std::uint32_t(nodes.size()),3,1};
    in.current.units=mesh.units==s::Coordinates::Native?l::KinematicsUnits::Native:l::KinematicsUnits::Si;in.current.native_units=mesh.scale;
    in.accepted_rows=&history;in.accepted_row_count=1;in.spatial=raw.empty()?nullptr:raw.data();in.spatial_count=raw.size();
    in.spatial_by_secondary={raw_offsets,2,raw_order.empty()?nullptr:raw_order.data(),raw_order.size()};
    if(l::detail::Validate(in)!=n::selection::Status::Ok)throw std::runtime_error("Normal fixture lifecycle source rejected");
    n::units_detail::Factors units;if(!l::detail::Factors(in.current,units))throw std::runtime_error("Normal fixture unit boundary rejected");
    const auto row=l::detail::PrepareRowBeforeNormals(in,0,units);std::vector<std::uint32_t> optimized;
    for(auto main:raw_mains)if(l::detail::OptimizedCandidate(in,0,int(main),row.optimization_main,row.optimization_leave,units))optimized.push_back(main);
    n::normal_activation::Input activation{{0,0,1,2,1,n::normal_activation::FreeRosterPolicy::FreshComplete},in.source,&row,1,
      optimized.empty()?nullptr:optimized.data(),optimized.size(),free_ids.empty()?nullptr:free_ids.data(),free_ids.size()};
    if(n::normal_activation::EvaluateNativeNormalActivation(activation,{},
      {main_active.data(),main_active.size(),node_tag.data(),node_tag.size()})!=n::selection::Status::Ok)
      throw std::runtime_error("Normal fixture native activation producer rejected");
  }
  static c::Limits Limits(){return {256,160,1280,1280,8u<<20};}
};

using Fixture=FixtureT<>;

inline Case CornerFanCase() {
  Case fan;fan.ids={1,2,3,4,5};fan.positions={0,0,0,1,0,0,0,1,0,-1,0,0,0,-1,0};
  fan.Add(n::ShellLayout::Triangle3,0,1,2,2);fan.Add(n::ShellLayout::Triangle3,0,3,4,4);return fan;
}
// Native-only topology packets: expected normals still come exclusively from
// original NORMP. Such numerical inputs do not grant source-factory or topology
// refresh authority. Keep the unmodified original native result separately.
struct NativePacket {
  Case mesh;
  type25_startup_test::NativeResult native_start;
  std::vector<s::Main> mains;
  std::vector<std::uint32_t> active,tag,free_ids;
  explicit NativePacket(Case source):mesh(std::move(source)),native_start(type25_startup_test::Oracle(mesh.Input(),mesh.coefficients.data(),mesh.coefficients.size())),
      mains(native_start.mains),active(mains.size(),1),tag(mesh.ids.size(),1){RefreshFree();}
  void RefreshFree() {
    free_ids.clear();for(std::size_t m=0;m<mains.size();++m)for(unsigned k=0;k<4;++k)
      if(!mains[m].neighbors[k]&&!(k==2&&mains[m].nodes[2]==mains[m].nodes[3])){
        free_ids.push_back(std::uint32_t(m+1));break;}
  }
  c::Input Input() const {
    const auto& data=native_start;c::Input in;in.profile=c::Profile::OrdinaryShellLocal;in.free_roster=n::normal_activation::FreeRosterPolicy::FreshComplete;
    in.topology={mains.data(),mesh.ids.size(),mesh.primary.size(),mains.size(),data.starter_references.size(),
      {data.offsets.data(),data.offsets.size(),data.incidence.data(),data.incidence.size()}};
    in.positions=mesh.Input().positions;in.main_coefficients=mesh.coefficients.data();in.coefficient_count=mesh.coefficients.size();
    in.main_active=active.data();in.active_count=active.size();in.node_tag=tag.data();in.tag_count=tag.size();
    in.free_main_ids=free_ids.data();in.free_count=free_ids.size();in.prior_normals=data.starter_normals.data();in.prior_count=data.starter_normals.size();return in;
  }
};
struct NativeCornerFan:NativePacket {
  NativeCornerFan():NativePacket(CornerFanCase()){}
};
struct NativeOpenEdges:NativePacket {
  NativeOpenEdges():NativePacket(Grid(2,2)) {
    // Explicit changed-topology numerical packet. Clear BOTH directions of all
    // interior links while retaining original node-bound reference groups/CSR.
    // This is not a claimed native deletion or refresh-lifecycle implementation.
    for(auto& main:mains)for(unsigned k=0;k<4;++k){main.neighbors[k]=0;main.neighbor_edges[k]=0;}
    RefreshFree(); // Original FREE_BOUND independently verifies this input list.
  }
};

struct Result {c::Report report;std::vector<n::StoredNormal> normals;std::vector<s::NormalReference> references;};
inline Result EvaluateHostNormals(const c::Input& in,c::Limits limits=Fixture::Limits()) {
  c::Forecast forecast;const auto admitted=c::Preflight(in,limits,forecast);if(admitted.status!=c::Status::Ok)return {admitted,{},{}};
  tl::util::HostArena scratch;if(!scratch.Initialize(forecast.scratch_bytes))throw std::runtime_error("Normal fixture scratch allocation failed");
  Result out;out.normals.resize(4*in.topology.main_count);out.references.resize(in.topology.references);
  out.report=c::Evaluate(in,limits,scratch.data(),scratch.bytes(),{out.normals.data(),out.normals.size(),out.references.data(),out.references.size()});return out;
}
}
