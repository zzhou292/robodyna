// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Source.h"
#include "lib_src/collision/RadiossType25InitialState.h"
#include "../SourceAdmissionFixture.h"
#include "lib_src/collision/radioss_type25/activity_source/Types.h"
namespace glass_removal_test {
namespace n=tlfea::contact::radioss_type25;
namespace s=n::startup;namespace l=n::lifecycle;
struct ContactSource {
 std::vector<std::uint64_t> ids,parents;
 std::vector<s::PrimaryFace> primary;
 std::vector<l::Node> nodes;std::vector<l::Main> mains;
 std::vector<l::Secondary> secondary;std::vector<l::NormalReference> references;
 std::vector<double> curvature,coefficients;
 std::vector<std::uint32_t> removal_offsets;
 tl::util::HostArena arena,scratch,ready_arena,ready_scratch;
 s::Snapshot starter;s::FixedMainView ready;
 n::initial_source::PreparedSource initial;
 void PrepareInitial(const nodal_empty_test::Fixture&,const n::TransactionConfig&);
 n::activity_source::Controls controls;
 std::uint64_t id=0;
 void Initialize(const nodal_empty_test::Fixture& physical,const nodal_empty_test::Source& mesh,bool wall) {
  id=wall?910201:910200;
  controls={wall?n::activity_source::Deletion::Disabled:n::activity_source::Deletion::ContainingElement,
    false,s::SolidErosion::Disabled};
  for(std::size_t i=0;i<physical.domain.node_count();++i){
   const auto nid=physical.domain.nodes()[i].source_id;ids.push_back(nid);
   nodes.push_back({nid,physical.fixed[i],0});
   if(i>=4)secondary.push_back({std::uint32_t(i),1e6,.001,0});
  }
  const auto add=[&](const auto& source,n::ShellLayout layout){
   s::PrimaryFace face;face.source_id=source.source_parent_id;face.layout=layout;
   for(unsigned i=0;i<4;++i){const auto slot=layout==n::ShellLayout::Triangle3&&i==3?2:i;
    face.nodes[i]=std::uint32_t(physical.domain.Find(source.reference.node_ids[slot]));}
   primary.push_back(face);parents.push_back(face.source_id);
  };
  if(wall)add(mesh.quads[0],n::ShellLayout::Quad4);
  else {add(mesh.quads[1],n::ShellLayout::Quad4);add(mesh.triangles[0],n::ShellLayout::Triangle3);}
  s::Input input;input.profile=wall?s::Profile::OrdinaryExteriorFixedMain:s::Profile::OrdinaryExteriorMovingMain;
  input.topology=wall?s::TopologyPolicy::ManifoldTwoSided:s::TopologyPolicy::NativeOrdinaryShell;
  input.node_source_ids=ids.data();input.node_count=ids.size();
  input.positions={physical.positions.data(),std::uint32_t(ids.size()),3,1};
  input.primary=primary.data();input.primary_count=primary.size();input.source_generation=7;
  const auto f=s::Preflight(input);
  physical.Require(f.status==s::Status::Ok&&arena.Initialize(f.output_bytes)&&scratch.Initialize(f.scratch_bytes),"Glass contact topology allocation");
  physical.Require(s::BuildStarter(input,{},arena,scratch,&starter).status==s::Status::Ok,"Glass contact topology");
  coefficients.assign(starter.main_count,1e6);
  if(wall){
   physical.Require(ready_arena.Initialize(f.ready_output_bytes)&&ready_scratch.Initialize(f.ready_scratch_bytes),"Glass wall ready allocation");
   physical.Require(s::BuildFixedMain(input,starter,{coefficients.data(),coefficients.size()},{},ready_arena,ready_scratch,&ready).status==s::Status::Ok,"Fixed mesh wall normals");
  }
  const auto normals=wall?ready.normals:starter.starter;
  mains.resize(starter.main_count);references.resize(normals.reference_count);
  for(std::size_t i=0;i<mains.size();++i){const auto& original=starter.mains[i];auto& out=mains[i];
   out.global_id=original.global_id;out.segment_type=original.segment_type;out.coefficient=1e6;out.maximum_gap=.001;
   for(unsigned k=0;k<4;++k){out.nodes[k]=original.nodes[k];out.neighbors[k]=original.neighbors[k];
    out.normal_reference[k]=original.normal_reference[k];out.normal_slot[k]=normals.face_normals[4*i+k];out.gap[k]=.001;}
  }
  for(std::size_t i=0;i<references.size();++i){references[i].boundary=normals.references[i].boundary;
   for(unsigned k=0;k<2;++k)references[i].bisector[k]=normals.references[i].bisector[k];}
  curvature.assign(primary.size(),0);removal_offsets.assign(secondary.size()+1,0);
 }
 n::FixedMainSource Common() const {
  n::FixedMainSource out;out.source_id=id;out.topology_generation=3;
  out.selection={nodes.data(),nodes.size(),mains.data(),mains.size(),secondary.data(),secondary.size(),
   references.data(),references.size(),{starter.normal_offsets,references.size()+1,starter.normal_mains,starter.normal_incidence_count},
   {removal_offsets.data(),removal_offsets.size(),nullptr,0},7};
  out.primary_main_count=primary.size();out.primary_parent_ids=parents.data();out.primary_curvature=curvature.data();
  out.margin=initial.prepared()?initial.removals().engine_margin:.01;
  if(initial.prepared()){
   out.primary_curvature=initial.removals().primary_extent;
   out.selection.removed_main_by_secondary={}; // The genuine producer supplies its exact finalized CSR.
  }
  out.force_packet_size=128;out.native_workers=1;out.contact_thickness_update=0;out.activity_controls=&controls;return out;
 }
 n::MovingMainSource Moving() const {
  n::MovingMainSource out;static_cast<n::ContactSourceInput&>(out)=Common();out.starter=starter;
  out.activation={0,0,1,2,1,n::normal_activation::FreeRosterPolicy::FreshComplete};return out;
 }
};
}
