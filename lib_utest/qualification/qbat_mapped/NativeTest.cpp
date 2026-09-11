// SPDX-License-Identifier: MIT
#include "../qbat_resident/Fixture.h"
#include "../qbat_force/NativeOracle.h"
#include "lib_src/elements/qbat/mapped/Stiffness.h"

extern "C" void qbm_native_initial(const double*,const double*,const double*,const double*,const double*,double*);
extern "C" void qeph_q2_scatter(const double*,const double*,const int*,double*);
namespace qbat_mapped_test {
namespace qb=tl::fea::qbat;
namespace mapped=qb::mapped;
namespace batch=qb::batch_detail;
void CheckScatter(const qb::BatchResult& result,const mapped::NodalStiffness& stiffness,
    const double (&coefficients)[4],const double* native_force=nullptr) {
  double internal[24]{};
  for (unsigned slot=0;slot<4;++slot) {
    const auto force=result.internal_force_n[slot];
    const auto couple=result.internal_couple_nm[slot];
    internal[3*slot]=force.x;internal[3*slot+1]=force.y;internal[3*slot+2]=force.z;
    internal[12+3*slot]=couple.x;internal[13+3*slot]=couple.y;internal[14+3*slot]=couple.z;
  }
  const int native_nodes[]{4,2,1,3};
  const std::size_t nodes[]{3,1,0,2};
  double output[32]{};
  // The existing wrapper calls the complete authenticated CUPDTN3, with
  // off>=0 bookkeeping only. Final force/OFF masking is tested by QBF itself.
  qeph_q2_scatter(native_force?native_force:internal,coefficients,native_nodes,output);
  double translation[4]{},rotation[4]{};
  ASSERT_TRUE(mapped::AddStiffness(nodes,stiffness,translation,rotation,4));
  for (unsigned slot=0;slot<4;++slot) {
    qbat_force_test::Close(translation[slot],output[24+slot],2e-11);
    EXPECT_EQ(rotation[slot],output[28+slot]);
    for (unsigned axis=0;axis<3;++axis) {
      qbat_force_test::Close(-internal[3*slot+axis],output[3*nodes[slot]+axis],2e-9);
      qbat_force_test::Close(-internal[12+3*slot+axis],output[12+3*nodes[slot]+axis],2e-9);
    }
  }
}
TEST(QbatMappedNative,VirginCurrentCoefficientsAndCompleteNativeScatter) {
  for (bool warped:{false,true}) for (double dm:{0.,.013}) {
    qbat_force_test::Fixture f(warped,dm);
    const auto element=qbat_resident_test::Element(f);
    qb::BatchResult virgin;
    ASSERT_EQ(batch::InitializeResult(element,virgin),qb::Status::kSuccess);
    mapped::NodalStiffness stiffness;
    ASSERT_TRUE(mapped::AcceptedStiffness(element,virgin,stiffness));
    double x[12],coefficients[4];
    for (unsigned slot=0;slot<4;++slot) {
      const auto p=f.input.quadrilateral.position[slot];
      x[3*slot]=p.x;x[3*slot+1]=p.y;x[3*slot+2]=p.z;
    }
    qbm_native_initial(x,&virgin.history.thickness_m,&f.material.a11,
        &f.input.options.membrane_viscosity,&f.input.options.numerical_viscosity,coefficients);
    CheckScatter(virgin,stiffness,coefficients);
    EXPECT_EQ(virgin.stamp.sample_index,0u);
    EXPECT_EQ(virgin.diagnostics.translation_stiffness_n_m,0);
    EXPECT_EQ(coefficients[1],0);
  }
}
TEST(QbatMappedNative,IndependentRecurrentNativeForceStiffnessAndRemoval) {
  for (double d1:{2.5,.002}) {
    qbat_force_test::Fixture f(true,.013);
    f.failure.failure_strain=d1;
    const auto element=qbat_resident_test::Element(f);
    qb::BatchResult accepted;
    ASSERT_EQ(batch::InitializeResult(element,accepted),qb::Status::kSuccess);
    qbat_force_test::NativeState native(accepted.history);
    bool removed=false;
    for (unsigned step=0;step<160;++step) {
      SCOPED_TRACE(step);
      qb::BatchResult next;
      const auto interval=qbat_force_test::Path(f,step);
      ASSERT_EQ(batch::Advance(element,accepted,interval,next),qb::Status::kSuccess);
      native.Step(f,interval);
      qbat_force_test::CompareNative(qbat_resident_test::Restore(element,next),native);
      mapped::NodalStiffness stiffness;
      ASSERT_TRUE(mapped::AcceptedStiffness(element,next,stiffness));
      const double coefficients[]{native.output[264],native.output[265],native.output[258],native.output[259]};
      CheckScatter(next,stiffness,coefficients,native.output.data());
      EXPECT_EQ(next.history.element_active,native.state[115]!=0);
      removed=removed || !next.history.element_active;
      accepted=next;
    }
    if (d1<1) EXPECT_TRUE(removed);
  }
}
} // namespace qbat_mapped_test
