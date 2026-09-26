// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "FullLedgerFixture.h"
#include "lib_src/collision/radioss_type25/runtime/PhysicalMainSource.h"
#include "lib_src/collision/radioss_type25/runtime/physical_main/Origins.h"
#include "lib_src/collision/radioss_type25/source_surfaces/Internal.h"
namespace type25_physical_main_test {
namespace n=tlfea::contact::radioss_type25;namespace s=n::startup;
namespace rd=n::runtime_detail;namespace pm=rd::physical_main;
struct Fixture {
  type25_source_test::FullLedgerFixture physical{true};
  pm::Index index;
  std::array<s::PrimaryFace,3> primary;
  std::array<s::PrimaryFaceIdentity,3> identities;
  std::array<s::PrimaryFaceIdentity,4> origins;
  std::array<std::uint32_t,4> origin_map{{0,1,2,2}};
  std::array<s::PrimaryCornerPermutation,3> permutations;
  std::array<s::PreShellSolidSupport,3> before{};
  std::array<s::PostGapmMainSupport,5> supports;
  tl::util::HostArena output,scratch;
  s::MixedSidesSnapshot sides;
  s::PostGapmTopology post;
  Fixture() {
    if(index.Initialize(physical.physical,1u<<20).status!=n::TransactionStatus::Ok)
      throw std::runtime_error("Actual physical source index failed");
    pm::Face q,tri;bool triangle=false;
    if(!index.Shell(103,q,triangle)||triangle||!index.Shell(102,tri,triangle)||!triangle)
      throw std::runtime_error("Missing actual shell family source");
    primary[0].source_id=103;primary[0].layout=n::ShellLayout::Quad4;
    primary[1].source_id=102;primary[1].layout=n::ShellLayout::Triangle3;
    for(unsigned k=0;k<4;++k){primary[0].nodes[k]=q[k];primary[1].nodes[k]=tri[k];}
    std::array<std::uint32_t,8> nodes;
    if(!index.Solid(9100,nodes))throw std::runtime_error("Missing actual solid source");
    n::source_surfaces::Solid solid;
    for(unsigned k=0;k<8;++k)solid.nodes[k]=nodes[k];
    if(n::source_surfaces::detail::CompactFace(solid,0,primary[2].nodes)!=4)
      throw std::runtime_error("Expected complete physical solid face");
    primary[2].layout=n::ShellLayout::Quad4;
    identities[0]={s::PrimaryFaceKind::Shell,103,0};
    identities[1]={s::PrimaryFaceKind::Shell,102,0};
    identities[2]={s::PrimaryFaceKind::Solid,0,0,s::PrimaryOrigin::MultipleOrigins,2};
    origins={identities[0],identities[1],s::PrimaryFaceIdentity{s::PrimaryFaceKind::Solid,9100,1},
      s::PrimaryFaceIdentity{s::PrimaryFaceKind::Solid,9101,1}};
    s::Input input;input.profile=s::Profile::MixedSurface;input.topology=s::TopologyPolicy::NativeMixedSurface;
    input.node_source_ids=physical.ids.data();input.node_count=physical.ids.size();
    input.positions={physical.positions.data(),std::uint32_t(physical.ids.size()),3,1};
    input.coordinates=s::Coordinates::Si;input.units={1,1,1};
    input.primary=primary.data();input.primary_count=3;input.source_generation=7;
    input.primary_identities=identities.data();input.primary_identity_count=3;input.shell_primary_count=2;
    input.raw_origins=origins.data();input.raw_origin_to_primary=origin_map.data();input.raw_origin_count=4;
    const auto forecast=s::PreflightMixedSides(input);
    if(forecast.status!=s::Status::Ok||!output.Initialize(forecast.output_bytes)||
        !scratch.Initialize(forecast.scratch_bytes)||s::BuildMixedSides(input,{},output,scratch,&sides).status!=s::Status::Ok)
      throw std::runtime_error("Actual mixed-side producer failed");
    permutations[1]={{0,1,2,2}};
    supports[0].first={s::PhysicalSupportKind::ShellQuad,103};supports[3]=supports[0];
    supports[1].first={s::PhysicalSupportKind::ShellTriangle,102};supports[4]=supports[1];
    supports[2]={{s::PhysicalSupportKind::EightSlotSolid,9101},9100};before[2]={9101,9100,2};
    post={s::PostGapmPhase::FinalizedBeforeNeighbors,permutations.data(),3,before.data(),3,supports.data(),5,
      1,s::SolidErosion::Enabled,s::SolidErosion::Enabled,7};
  }
  rd::PhysicalMainValidation Validate(std::size_t cap=1u<<20) const {
    return rd::ValidateMixedPhysicalMains(physical.physical,sides,post,cap);
  }
};
} // namespace type25_physical_main_test
