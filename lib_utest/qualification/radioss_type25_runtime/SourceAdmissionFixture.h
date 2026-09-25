// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../nodal_empty_cin/Fixture.h"
#include "lib_src/collision/RadiossType25FixedMainStartup.h"
#include "lib_src/collision/radioss_type25/runtime/Source.h"
namespace type25_source_test {
namespace n=tlfea::contact::radioss_type25;
namespace s=n::startup;namespace l=n::lifecycle;namespace rd=n::runtime_detail;
// Synthetic declared two-shell source: real TL mass/material/domain bindings,
// genuine Starter producer, no CUDA owner or captured native mechanics.
struct Fixture {
  nodal_empty_test::Fixture physical;
  std::vector<std::uint64_t> ids,parents;
  std::vector<s::PrimaryFace> primary;
  std::vector<l::Node> nodes;
  std::vector<l::Main> mains;
  std::vector<l::Secondary> secondary;
  std::vector<l::NormalReference> references;
  std::vector<double> curvature;
  std::vector<std::uint32_t> removal_offsets;
  tl::util::HostArena output,scratch;
  s::Snapshot starter;
  explicit Fixture(bool global_and_general=false) {
    auto mesh=nodal_empty_test::SmallSource();
    if(global_and_general) {
      auto& material=mesh.materials[0];material.law=tl::fea::ShellSectionLaw::LayeredLaw1Nip3;
      material.hardening=tl::material::ShellPlasticityHardeningKind::Tabulated;
      material.curve_id=0;material.linear={};material.rate={};
      for(auto& parent:mesh.parents)parent.execution={tl::fea::ShellParentExecutionPolicy::GlobalLaw1Npt0,
          {tl::fea::ShellLaw1Thickness::Accepted,1.}};
      for(auto& x:mesh.triangles[0].reference.position)x.z=.002000001;
    }
    physical.Prepare(mesh,{7,7,7,7,0,1,2},{1,1,1,1,0,1,0});
    for(std::size_t i=0;i<physical.domain.node_count();++i) {
      const auto id=physical.domain.nodes()[i].source_id;ids.push_back(id);
      nodes.push_back({id,0,0});secondary.push_back({std::uint32_t(i),1e6,.001,0});
    }
    for(const auto& q:mesh.quads) {
      s::PrimaryFace face;face.source_id=q.source_parent_id;face.layout=n::ShellLayout::Quad4;
      for(unsigned k=0;k<4;++k)face.nodes[k]=q.nodes[k];primary.push_back(face);parents.push_back(face.source_id);
    }
    for(const auto& t:mesh.triangles) {
      s::PrimaryFace face;face.source_id=t.source_parent_id;face.layout=n::ShellLayout::Triangle3;
      for(unsigned k=0;k<4;++k)face.nodes[k]=t.nodes[k<3?k:2];primary.push_back(face);parents.push_back(face.source_id);
    }
    s::Input input;input.profile=s::Profile::OrdinaryExteriorMovingMain;
    if(global_and_general)input.topology=s::TopologyPolicy::NativeOrdinaryShell;
    input.node_source_ids=ids.data();input.node_count=ids.size();
    input.positions={physical.positions.data(),std::uint32_t(ids.size()),3,1};
    input.primary=primary.data();input.primary_count=primary.size();input.source_generation=7;
    const auto forecast=s::Preflight(input);
    physical.Require(forecast.status==s::Status::Ok&&output.Initialize(forecast.output_bytes)&&
        scratch.Initialize(forecast.scratch_bytes),"Source fixture startup arenas");
    physical.Require(s::BuildStarter(input,{},output,scratch,&starter).status==s::Status::Ok,"Source fixture Starter");
    mains.resize(starter.main_count);references.resize(starter.starter.reference_count);
    for(std::size_t i=0;i<mains.size();++i) {
      const auto& original=starter.mains[i];auto& main=mains[i];
      main.global_id=original.global_id;main.segment_type=original.segment_type;
      main.coefficient=1e6;main.maximum_gap=.001;
      for(unsigned k=0;k<4;++k) {
        main.nodes[k]=original.nodes[k];main.normal_reference[k]=original.normal_reference[k];
        main.neighbors[k]=original.neighbors[k];main.normal_slot[k]=starter.starter.face_normals[4*i+k];main.gap[k]=.001;
      }
    }
    for(std::size_t i=0;i<references.size();++i) {
      const auto& original=starter.starter.references[i];references[i].boundary=original.boundary;
      for(unsigned k=0;k<2;++k)references[i].bisector[k]=original.bisector[k];
    }
    removal_offsets.assign(secondary.size()+1,0);curvature.assign(primary.size(),0);
  }
  n::MovingMainSource Source() const {
    n::MovingMainSource source;source.source_id=77;source.topology_generation=3;
    source.selection={nodes.data(),nodes.size(),mains.data(),mains.size(),secondary.data(),secondary.size(),
      references.data(),references.size(),{starter.normal_offsets,references.size()+1,starter.normal_mains,starter.normal_incidence_count},
      {removal_offsets.data(),removal_offsets.size(),nullptr,0},7};
    source.primary_main_count=primary.size();source.primary_parent_ids=parents.data();source.primary_curvature=curvature.data();
    source.margin=.01;source.force_packet_size=128;source.native_workers=1;
    source.starter=starter;source.activation={0,0,1,2,1,n::normal_activation::FreeRosterPolicy::FreshComplete};return source;
  }
  static n::TransactionConfig Config() {
    n::TransactionConfig c;c.units={1,1,1};c.lifecycle.selection={1,5,1,false,false,false};
    c.lifecycle.geometry={1,1,5,1,false,false,false};c.lifecycle.coefficient={4,0};
    c.lifecycle.neighbor_removal=2;c.lifecycle.optcd_response_precision=0;
    c.normal.engine={0,0,0};c.normal.damping_factor=.05;c.friction={2,10,0,1,0,0,1};
    c.friction_coefficients={.1,{0,0,0,0,.1,-.001}};c.assembly={0,0,0,0,0,{0,0,0}};return c;
  }
};
} // namespace type25_source_test
