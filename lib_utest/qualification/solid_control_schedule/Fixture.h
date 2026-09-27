// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_utest/qualification/extended_solid_model/Fixture.h"
#include "lib_src/elements/solids/control/Selection.h"
#include "lib_src/elements/solids/resident/Arena.h"
namespace control_schedule_test {
namespace s=tl::fea::solids;
namespace c=s::control;
namespace fe=tl::fea;
using solid_model_test::Require;
struct Fixture:extended_model_test::Fixture {
  s::Model base;
  std::vector<c::SourceParent> source;
  std::vector<c::NativePacket> packets;
  std::vector<c::NativePartition> partitions;
  std::vector<std::uint64_t> members;
  Fixture() {
    auto extra=input24[0];auto ref=extra.reference.input();ref.source_element_id=202;
    Require(fe::solid24::InitializeReference(ref,extra.reference)==fe::solid24::Status::Success);
    input24.push_back(extra);
    Require(bool(base.Initialize(Domain(),extended_model_test::Fixture::Input())));
    for(const auto& row:base.contributions()->parents()) {
      const bool enabled=row.family==s::Family::Solid24||row.family==s::Family::Solid6z||row.family==s::Family::Solid18Law90;
      source.push_back({row.source_element_id,row.source_part_id,row.source_section_id,row.source_material_id,row.source_section_id,enabled?1u:0u});
    }
    // Native packet/member order intentionally differs from TL family order.
    members={105,202,102,101,104,103};
    packets={
      {9,0,0,1,s::Family::Solid18Law90,90,19,1},
      {10,1,1,2,s::Family::Solid24,30,13,1},
      {11,3,3,1,s::Family::Solid18,20,11,0},
      {12,4,4,1,s::Family::Solid18Law44,44,17,0},
      {13,5,5,1,s::Family::Solid6z,30,15,1}};
    partitions={{0,0,5,0,6}};
    std::reverse(source.begin(),source.end());
  }
  c::Input Controls()const {
    c::Input value;value.profile=c::Profile::SourceDeclared;value.source_instance_id=777;
    value.units={.001,1000,1};value.native_nvsiz=128;value.compiled_mvsiz=129;
    value.parents={source.data(),source.size()};value.packets={packets.data(),packets.size()};
    value.partitions={partitions.data(),partitions.size()};value.ordered_element_ids={members.data(),members.size()};return value;
  }
  s::ModelInput ControlledInput()const {
    auto value=extended_model_test::Fixture::Input();value.controls=Controls();return value;
  }
};
inline c::Report Initialize(const Fixture& f,c::Selection& s){return s.Initialize(*f.base.contributions(),f.Controls());}
} // namespace control_schedule_test
