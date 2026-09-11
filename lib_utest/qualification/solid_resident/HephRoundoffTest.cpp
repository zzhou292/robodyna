// SPDX-License-Identifier: AGPL-3.0-or-later
#include "HephRoundoff.h"
#include "HephCapturedPacket.h"
#include "../solid24_force/TestSupport.h"
#include "../solid24_force/NativeComparison.h"
#include <gtest/gtest.h>

namespace solid_resident_test {
namespace capture = heph_capture;
namespace heph = tl::fea::solid24;

heph::Reference CapturedReference() {
  heph::ReferenceInput input;
  input.source_element_id=9101; input.source_part_id=9203;
  input.source_section_id=9204; input.source_material_id=9205;
  input.density_kg_m3=1980;
  const tl::math::Vec3 x[8]{{0,0,0},{.04,0,0},{.04,.03,0},{0,.03,0},
      {0,0,.02},{.04,0,.02},{.04,.03,.02},{0,.03,.02}};
  for (unsigned n=0; n<8; ++n) {
    input.source_node_id[n]=n<2 ? 10+n : 9300+n;
    input.position_m[n]=x[n];
  }
  return heph_test::Reference(input);
}

TEST(SolidResidentRoundoff, ExactOwnerPacketSeparatesSpectralCancellationFromGeometry) {
  const auto reference=CapturedReference();
  const auto material=heph_test::Material();
  heph::ForceTrial initial,trial;
  ASSERT_EQ(heph::InitializeForce(reference,material,{},initial),heph::ForceStatus::Success);
  auto interval=heph_test::Interval(reference,initial.proposed_history,0x1p-20);
  for (unsigned n=0; n<8; ++n) {
    interval.position_m[n]={capture::Position[3*n],capture::Position[3*n+1],capture::Position[3*n+2]};
    interval.velocity_m_s[n]={capture::Velocity[3*n],capture::Velocity[3*n+1],capture::Velocity[3*n+2]};
  }
  ASSERT_EQ(heph::EvaluateForce(reference,initial.proposed_history,interval,material,trial),
      heph::ForceStatus::Success);
  const auto values=heph_test::Values(trial);
  HephRoundoff bound;
  ASSERT_TRUE(PrepareHephRoundoff(material,capture::Native,capture::Initial,
      reference.geometry().volume_m3,interval.dt_s,{},bound));
  for (unsigned k=46; k<151; ++k) EXPECT_EQ(values[k],capture::Native[k])<<k;
  for (unsigned k=173; k<179; ++k) EXPECT_EQ(values[k],capture::Native[k])<<k;
  for (unsigned k=0; k<values.size(); ++k) {
    EXPECT_TRUE(std::isfinite(values[k]));
    EXPECT_NEAR(values[k],capture::Native[k],
        heph_test::ForceTolerance(k,capture::Native)+bound.Additional(k))<<k;
  }
  // Both independent spectral implementations are near the invariant result;
  // the previous residual-relative criterion demonstrably rejects the packet.
  for (unsigned k=0; k<6; ++k) {
    EXPECT_NEAR(values[k],capture::InvariantStress[k],bound.stress_pa)<<k;
    EXPECT_NEAR(capture::Native[k],capture::InvariantStress[k],bound.stress_pa)<<k;
  }
  for (unsigned k=0; k<3; ++k) {
    EXPECT_GT(std::abs(values[k]-capture::Native[k]),heph_test::ForceTolerance(k,capture::Native));
    EXPECT_NEAR(values[k]-capture::Native[k],-0x1p-28,4*std::numeric_limits<double>::epsilon());
  }
}

TEST(SolidResidentRoundoff, OnlyStressAndItsForceWorkConsequencesReceiveAnAllowance) {
  HephRoundoff bound;
  const auto reference=CapturedReference();
  ASSERT_TRUE(PrepareHephRoundoff(heph_test::Material(),capture::Native,capture::Initial,
      reference.geometry().volume_m3,0x1p-20,{},bound));
  const auto agrees=[&](unsigned k,double change) {
    const double actual=capture::Native[k]+change;
    return std::isfinite(actual) && std::abs(actual-capture::Native[k])<=
        heph_test::ForceTolerance(k,capture::Native)+bound.Additional(k);
  };
  for (unsigned k : {0u,1u,2u,3u,4u,5u,151u,156u,160u,165u,166u,167u}) {
    EXPECT_FALSE(agrees(k,.01))<<k;
    EXPECT_GT(bound.Additional(k),0)<<k;
  }
  for (unsigned k=22; k<46; ++k) EXPECT_FALSE(agrees(k,.001))<<k;
  for (unsigned k : {7u,158u,181u})
    EXPECT_FALSE(agrees(k,8*(heph_test::ForceTolerance(k,capture::Native)+bound.Additional(k))))<<k;
  for (unsigned k=0; k<187; ++k) {
    const bool receives=k<6 || k==7 || (k>=22 && k<46) || (k>=151 && k<157) ||
        k==158 || (k>=160 && k<168) || k==181;
    if (!receives) EXPECT_EQ(bound.Additional(k),0)<<k;
  }
  // Independent native physics checks retain the old criteria, including a
  // real nonzero HG-viscosity coefficient and exact active/phase channels.
  EXPECT_GT(capture::Native[186],0);
  for (unsigned k : {6u,8u,9u,21u,103u,168u,169u,170u,171u,184u,185u,186u})
    EXPECT_FALSE(agrees(k,8*heph_test::ForceTolerance(k,capture::Native)))<<k;
}

TEST(SolidResidentRoundoff, CandidateBoundsRejectOutsideTheProvenDomainAndCarryOnlyAcceptedEnergy) {
  const auto material=heph_test::Material();
  const double volume=CapturedReference().geometry().volume_m3;
  HephRoundoff first,second;
  ASSERT_TRUE(PrepareHephRoundoff(material,capture::Native,capture::Initial,volume,0x1p-20,{},first));
  ASSERT_TRUE(PrepareHephRoundoff(material,capture::Native,capture::Initial,volume,0x1p-20,first,second));
  EXPECT_GT(second.material_work_j,first.material_work_j);
  EXPECT_GT(second.accepted_energy_j_m3,first.accepted_energy_j_m3);
  auto bad=capture::Native;
  bad[173]=.125;
  HephRoundoff staged=first;
  EXPECT_FALSE(PrepareHephRoundoff(material,bad,capture::Initial,volume,0x1p-20,first,staged));
  EXPECT_EQ(staged.stress_pa,first.stress_pa);
  EXPECT_EQ(staged.force_n,first.force_n);
  EXPECT_EQ(staged.material_work_j,first.material_work_j);
  EXPECT_EQ(staged.material_energy_j_m3,first.material_energy_j_m3);
  EXPECT_EQ(staged.accepted_energy_j_m3,first.accepted_energy_j_m3);
  // A retry with the same accepted budget reproduces the same allowance.
  ASSERT_TRUE(PrepareHephRoundoff(material,capture::Native,capture::Initial,volume,0x1p-20,first,staged));
  EXPECT_EQ(staged.material_work_j,second.material_work_j);
  EXPECT_EQ(staged.accepted_energy_j_m3,second.accepted_energy_j_m3);
}
} // namespace solid_resident_test
