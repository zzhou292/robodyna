// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ConstructorChecks.h"
#include "lib_utest/qualification/solid18_law44_startup/NativeOracle.h"
#include "lib_utest/qualification/law90_solid18_force/NativeSupport.h"
namespace extended_resident_test {
bool RearConstructor(const s::Parent18Law44& parent,const fe::solid18::law44::Material& material,
    fe::solid18::Vec3 velocity,const s::Result18Law44& result) {
  namespace law=fe::solid18::law44;
  const auto native=rear_startup_test::NativeInitialize(material,parent.reference.input(),velocity);
  law::ForceTrial direct;
  if(native.status||law::InitializeForce(parent.reference,material,velocity,direct)!=fe::solid18::Status::Success||
      !rear_force_test::Agree(direct,native))return false;
  if(law::PreparePrescribedHistory(parent.reference,material,result.history,result.stamp,
      direct.proposed_history)!=fe::solid18::Status::Success)return false;
  direct.diagnostics=result.cache.diagnostics;
  for(unsigned n=0;n<8;++n)direct.rhs_force_n[n]=result.cache.rhs_force_n[n];
  EXPECT_EQ(result.stamp.sample_index,0u);EXPECT_EQ(result.stamp.time_s,0);
  EXPECT_EQ(result.cache.stiffness.translation_n_m,.25*result.cache.diagnostics.raw_stiffness_n_m);
  EXPECT_EQ(result.cache.stiffness.rotation_nm,0);
  return rear_force_test::Agree(direct,native);
}
bool FoamConstructor(const s::Parent18Law90& parent,const tl::material::law90::PreparedMaterial& material,
    const tl::material::law90::PreparationInput& input,fe::solid18::Vec3 velocity,const s::Result18Law90& result) {
  namespace law=fe::solid18::total_strain;
  namespace test=law90_force_test;
  const auto prepared=law90_point_test::NativePrepared(input,material.curve());
  test::NativeForce native(prepared[1]);fe::solid18::PrescribedInterval initial;
  for(unsigned n=0;n<8;++n){initial.position_endpoint_m[n]=parent.reference.input().position_m[n];initial.velocity_midpoint_m_s[n]=velocity;}
  test::AdvanceNative(prepared.data(),material.curve(),parent.reference.input(),initial,true,native);
  law::ForceTrial direct;
  if(native.status||law::InitializeForce90(parent.reference,material,velocity,direct)!=fe::solid18::Status::Success||
      !test::ForceAgreement(test::ForceValues(direct),native))return false;
  if(law::PreparePrescribedHistory90(parent.reference,material,result.history,result.stamp,
      direct.proposed_history)!=fe::solid18::Status::Success)return false;
  direct.diagnostics=result.cache.diagnostics;
  for(unsigned n=0;n<8;++n) {
    direct.rhs_force_n[n]=result.cache.rhs_force_n[n];direct.point[n].material.history=result.history.point[n];
    for(unsigned k=0;k<3;++k)EXPECT_EQ(result.history.point[n].point.cursor[k],unsigned(native.cursor[3*n+k]));
  }
  EXPECT_EQ(result.stamp.sample_index,0u);EXPECT_EQ(result.stamp.time_s,0);
  EXPECT_EQ(result.cache.stiffness.translation_n_m,.25*result.cache.diagnostics.raw_stiffness_n_m);
  EXPECT_EQ(result.cache.stiffness.rotation_nm,0);
  return test::ForceAgreement(test::ForceValues(direct),native);
}
} // namespace extended_resident_test
