// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/constraints/NodalRigidAssemblyBinding.h"
#include "../rigid_part_model/SolidFixture.h"

namespace rigid_binding_test {
namespace fe=tl::fea;
namespace r=fe::rigid;
using S=fe::RigidBindingStatus;
struct Fixture {
  rigid_part_solid_test::Fixture solid{0};
  std::array<fe::NodalRigidGroupMember,2> plain_members{};
  std::array<std::uint64_t,2> other{10,11};
  std::unique_ptr<r::NodalRigidPartTopology> Topology(bool missing=false) const {
    const auto& ids=solid.ids;
    const r::PartTopologyPartInput part{200,ids.data(),ids.size()};
    r::PartTopologyInput in;
    in.source_instance_id=1;in.parts=&part;in.part_count=1;
    in.expected_members=ids.data();in.expected_member_count=ids.size();
    in.other_rigid_members=other.data();in.other_rigid_member_count=missing?1:2;
    auto out=std::make_unique<r::NodalRigidPartTopology>();
    EXPECT_TRUE(out->Initialize(in));return out;
  }
  Fixture() {
    for(unsigned k=0;k<2;++k) {
      const auto n=solid.domain.Find(other[k]);
      const auto& c=solid.ledger.nodes()[n].coefficients;
      plain_members[k]={other[k],n,solid.domain.nodes()[n].position,c.mass,c.isotropic_inertia,
        c.shell.physical_inertia,c.shell.added_inertia};
    }
  }
  std::unique_ptr<fe::NodalRigidGroupModel> Plain() const {
    const fe::NodalRigidGroupInput group{200,501,plain_members.data(),plain_members.size()};
    auto out=std::make_unique<fe::NodalRigidGroupModel>();
    EXPECT_TRUE(out->Initialize({29,solid.domain.node_count(),&group,1,{1000,.001}}));
    return out;
  }
  r::NodalRigidPartAssemblyModel Parts(bool missing=false) const {
    auto top=Topology(missing);
    r::NodalRigidPartAssemblyModel out;
    EXPECT_TRUE(out.Initialize(*top,solid.ledger,{1000,.001}));return out;
  }
};
inline void SameFrame(const r::PrincipalFrame& a,const r::PrincipalFrame& b) {
  for(unsigned k=0;k<9;++k)EXPECT_EQ(a.axes.v[k],b.axes.v[k]);
  EXPECT_EQ(a.inertia.x,b.inertia.x);EXPECT_EQ(a.inertia.y,b.inertia.y);EXPECT_EQ(a.inertia.z,b.inertia.z);
}
} // namespace rigid_binding_test
