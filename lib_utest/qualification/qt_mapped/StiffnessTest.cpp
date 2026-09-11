// SPDX-License-Identifier: MIT
#include "lib_src/elements/ShellNodalStiffness.h"
#include "lib_src/elements/qeph/mapped/Stiffness.h"
#include "lib_src/elements/t3/mapped/Stiffness.h"
#include "lib_src/elements/qeph/QephStartup.h"
#include "../qbat_binding/Fixture.h"
#include <limits>

namespace qt_mapped_test {
namespace fe=tl::fea;
using qbat_binding_test::Bytes;
using Law=fe::ShellSectionLaw;

TEST(QtMappedStiffness,VirginSourcePlacementAndOnePointRolesAreDistinct) {
  qbat_binding_test::Fixture source;
  fe::qeph::ReferenceData q;
  ASSERT_EQ(fe::qeph::InitializeReference(source.q[0].reference,q),fe::qeph::Status::kSuccess);
  fe::qeph::mapped::NodalStiffness placed;
  ASSERT_TRUE(fe::qeph::mapped::InitialStiffness(q,Law::LayeredLaw44Nip3,placed));
  for (unsigned slot=0;slot<4;++slot) {
    EXPECT_GT(placed.translation[slot],0);
    EXPECT_GT(placed.rotation[slot],0);
  }
  const auto before=Bytes(placed);
  EXPECT_FALSE(fe::qeph::mapped::InitialStiffness(q,Law::LayeredLaw1Nip3,placed));
  EXPECT_EQ(Bytes(placed),before);
  fe::t3::ReferenceData t;
  ASSERT_EQ(fe::t3::InitializeReference(source.t.reference,t),fe::t3::Status::kSuccess);
  fe::t3::mapped::NodalStiffness nip1,nip3;
  ASSERT_TRUE(fe::t3::mapped::InitialStiffness(t,Law::Law44Nip1,nip1));
  ASSERT_TRUE(fe::t3::mapped::InitialStiffness(t,Law::LayeredLaw44Nip3,nip3));
  EXPECT_GT(nip1.rotation[0],0);
  EXPECT_GT(nip3.rotation[0],nip1.rotation[0]);
  for (unsigned slot=1;slot<3;++slot) {
    EXPECT_EQ(nip1.translation[slot],nip1.translation[0]);
    EXPECT_EQ(nip1.rotation[slot],nip1.rotation[0]);
  }
}

TEST(QtMappedStiffness,CurrentAndRemovedCachesRetainNativeSlotFactors) {
  fe::qeph::ForceTrial q;
  q.diagnostics.translational_stiffness=37;
  q.diagnostics.rotational_stiffness=13;
  q.kinematics.nodal_factors[0]=.7;
  q.kinematics.nodal_factors[1]=1;
  fe::qeph::mapped::NodalStiffness packet;
  ASSERT_TRUE(fe::qeph::mapped::AcceptedStiffness(q,packet));
  EXPECT_EQ(packet.translation[0],37*.7);
  EXPECT_EQ(packet.translation[2],37*.7);
  EXPECT_EQ(packet.rotation[1],13);
  EXPECT_EQ(packet.rotation[3],13);
  q.diagnostics.translational_stiffness=0;
  q.diagnostics.rotational_stiffness=0;
  ASSERT_TRUE(fe::qeph::mapped::AcceptedStiffness(q,packet));
  for (unsigned slot=0;slot<4;++slot) {
    EXPECT_EQ(packet.translation[slot],0);
    EXPECT_EQ(packet.rotation[slot],0);
  }
  const auto before=Bytes(packet);
  q.kinematics.nodal_factors[1]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(fe::qeph::mapped::AcceptedStiffness(q,packet));
  EXPECT_EQ(Bytes(packet),before);
}

TEST(QtMappedStiffness,ThreeNodeScatterAndLateOverflowPreserveCompleteDestination) {
  fe::shell_nodal_stiffness::Packet<3> values{{5,5,5},{2,2,2}};
  const std::size_t nodes[]{3,0,2};
  std::array<double,4> translation{1,2,3,4},rotation{5,6,7,8};
  ASSERT_TRUE(fe::shell_nodal_stiffness::Add(nodes,values,translation.data(),rotation.data(),4));
  EXPECT_EQ(translation,(std::array<double,4>{6,2,8,9}));
  EXPECT_EQ(rotation,(std::array<double,4>{7,6,9,10}));
  values.rotation[2]=std::numeric_limits<double>::max();
  rotation[2]=std::numeric_limits<double>::max();
  const auto old_translation=translation,old_rotation=rotation;
  EXPECT_FALSE(fe::shell_nodal_stiffness::Add(nodes,values,translation.data(),rotation.data(),4));
  EXPECT_EQ(translation,old_translation);
  EXPECT_EQ(rotation,old_rotation);
  values.rotation[2]=0;
  ASSERT_TRUE(fe::shell_nodal_stiffness::Add(nodes,values,translation.data(),rotation.data(),4));
}
} // namespace qt_mapped_test
