// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "FullLedgerFixture.h"
#include "lib_src/collision/radioss_type25/runtime/PhysicalMainSource.h"
#include "lib_src/collision/radioss_type25/runtime/physical_main/Origins.h"
#include "lib_src/collision/radioss_type25/source_surfaces/Internal.h"
namespace type25_source_test {
// Declared synthetic mixed contact selection on the actual heterogeneous model.
// Two distinct co-located solids really share the internal raw face. This
// fixture supplies explicit support operands; it does not claim deck parsing.
struct MixedRuntimeSource {
  FullLedgerFixture& physical;
  std::array<s::PrimaryFaceIdentity,3> identities;
  std::array<s::PrimaryFaceIdentity,4> origins;
  std::array<std::uint32_t,4> origin_map{{0,1,2,2}};
  std::array<s::PrimaryCornerPermutation,3> corners;
  std::array<s::PreShellSolidSupport,3> before{};
  std::array<s::PostGapmMainSupport,5> supports;
  tl::util::HostArena side_output,side_scratch,output,scratch;
  s::MixedSidesSnapshot sides;s::PostGapmTopology post;s::Snapshot starter;
  explicit MixedRuntimeSource(FullLedgerFixture& value):physical(value) {
    namespace pm=n::runtime_detail::physical_main;
    pm::Index index;auto status=index.Initialize(physical.physical,1u<<20);
    if(status.status!=n::TransactionStatus::Ok)throw std::runtime_error(status.message);
    if(physical.primary.size()!=2||physical.primary[0].source_id!=103||physical.primary[1].source_id!=102)
      throw std::runtime_error("Mixed source needs genuine QBAT/T3 base selection");
    s::PrimaryFace solid;solid.layout=n::ShellLayout::Quad4;
    std::array<std::uint32_t,8> nodes;
    if(!index.Solid(9100,nodes))throw std::runtime_error("Missing declared physical solid");
    n::source_surfaces::Solid raw;for(unsigned k=0;k<8;++k)raw.nodes[k]=nodes[k];
    if(n::source_surfaces::detail::CompactFace(raw,0,solid.nodes)!=4)
      throw std::runtime_error("Missing original solid raw face");
    physical.primary.push_back(solid);
    identities={s::PrimaryFaceIdentity{s::PrimaryFaceKind::Shell,103,0},
      s::PrimaryFaceIdentity{s::PrimaryFaceKind::Shell,102,0},
      s::PrimaryFaceIdentity{s::PrimaryFaceKind::Solid,0,0,s::PrimaryOrigin::MultipleOrigins,2}};
    origins={identities[0],identities[1],s::PrimaryFaceIdentity{s::PrimaryFaceKind::Solid,9100,1},
      s::PrimaryFaceIdentity{s::PrimaryFaceKind::Solid,9101,1}};
    s::Input in;in.profile=s::Profile::MixedSurface;in.topology=s::TopologyPolicy::NativeMixedSurface;
    in.node_source_ids=physical.ids.data();in.node_count=physical.ids.size();
    in.positions={physical.positions.data(),std::uint32_t(physical.ids.size()),3,1};
    in.primary=physical.primary.data();in.primary_count=3;in.source_generation=7;
    in.primary_identities=identities.data();in.primary_identity_count=3;in.shell_primary_count=2;
    in.raw_origins=origins.data();in.raw_origin_to_primary=origin_map.data();in.raw_origin_count=4;
    const auto shape=s::PreflightMixedSides(in);
    Need(shape.status==s::Status::Ok&&side_output.Initialize(shape.output_bytes)&&side_scratch.Initialize(shape.scratch_bytes),"Mixed sides allocation");
    Need(s::BuildMixedSides(in,{},side_output,side_scratch,&sides).status==s::Status::Ok,"Mixed sides construction");
    corners[1]={{0,1,2,2}};
    supports[0].first={s::PhysicalSupportKind::ShellQuad,103};supports[3]=supports[0];
    supports[1].first={s::PhysicalSupportKind::ShellTriangle,102};supports[4]=supports[1];
    supports[2]={{s::PhysicalSupportKind::EightSlotSolid,9101},9100};before[2]={9101,9100,2};
    post={s::PostGapmPhase::FinalizedBeforeNeighbors,corners.data(),3,before.data(),3,supports.data(),5,
      1,s::SolidErosion::Enabled,s::SolidErosion::Enabled,7};
    const auto forecast=s::PreflightMixedStarter(in,sides,post);
    Need(forecast.status==s::Status::Ok&&output.Initialize(forecast.output_bytes)&&scratch.Initialize(forecast.scratch_bytes),"Mixed Starter allocation");
    Need(s::BuildStarter(in,sides,post,{},output,scratch,&starter).status==s::Status::Ok,"Mixed Starter construction");
    physical.starter=starter;physical.curvature.assign(3,0);
    physical.mains.resize(5);physical.references.resize(starter.starter.reference_count);
    for(std::size_t i=0;i<5;++i) {
      const auto& original=starter.mains[i];auto& main=physical.mains[i];
      main.global_id=original.global_id;main.segment_type=original.segment_type;
      main.coefficient=i==2?-1e6:1e6;main.maximum_gap=.001;
      for(unsigned k=0;k<4;++k) {
        main.nodes[k]=original.nodes[k];main.normal_reference[k]=original.normal_reference[k];
        main.neighbors[k]=original.neighbors[k];main.normal_slot[k]=starter.starter.face_normals[4*i+k];main.gap[k]=.001;
      }
    }
    for(std::size_t i=0;i<physical.references.size();++i) {
      const auto& original=starter.starter.references[i];auto& target=physical.references[i];
      target.boundary=original.boundary;for(unsigned k=0;k<2;++k)target.bisector[k]=original.bisector[k];
    }
  }
  static void Need(bool value,const char* message){if(!value)throw std::runtime_error(message);}
  n::TransactionConfig Config() const {
    auto c=physical.Config();c.activity=n::ContactActivityPolicy::AllActivePrefix;
    c.lifecycle.main_coefficient_domain=n::MainCoefficientDomain::NativeSigned;return c;
  }
  n::MixedMovingMainSource Source() const {
    n::MixedMovingMainSource result;static_cast<n::ContactSourceInput&>(result)=physical.Contact();
    result.primary_parent_ids=nullptr;result.contact_thickness_update=0;result.starter=starter;
    result.activation={0,0,1,2,1,n::normal_activation::FreeRosterPolicy::FreshComplete};return result;
  }
};
}
