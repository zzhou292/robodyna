// SPDX-License-Identifier: MIT
#include "../t3_one_point/NativeOracle.h"
#include "lib_src/elements/t3/mapped/Stiffness.h"
#include "NativeScatter.h"

namespace qt_mapped_test {
namespace point=t3_one_point_test;
namespace t=tl::fea::t3;
TEST(QtMappedNative, OriginalOnePointTriangleKeepsPositiveRotationalStiffness) {
  point::Fixture source;
  auto virgin=source.Virgin();
  point::NativeState initial_native(virgin);
  t::PrescribedInterval stationary;
  stationary.dt=point::Dt;
  stationary.sample_index=1;
  for (unsigned slot=0;slot<3;++slot) stationary.position[slot]=source.reference.input.position[slot];
  ASSERT_EQ(initial_native.Step(source,stationary),0);
  t::mapped::NodalStiffness initial;
  ASSERT_TRUE(t::mapped::InitialStiffness(source.reference,tl::fea::ShellSectionLaw::Law44Nip1,initial));
  Near(initial.translation[0],initial_native.output[98]);
  Near(initial.rotation[0],initial_native.output[99]);
  EXPECT_GT(initial.rotation[0],0);
  EXPECT_EQ(initial_native.output[95],0); // Actual NIP1 DM.
  EXPECT_EQ(initial_native.output[97],0); // Actual NIP1 GS.
  for (bool near_failure:{false,true}) {
    auto accepted=near_failure?point::NearFailure(source):source.Virgin();
    point::NativeState native(accepted);
    bool removed=false;
    for (unsigned step=0;step<16;++step) {
      t::OnePointForceTrial trial;
      const auto input=point::Path(source,step);
      ASSERT_EQ(t::EvaluateOnePointLaw44Force(source.reference,source.material,source.failure,accepted,input,trial),t::Status::kSuccess);
      ASSERT_EQ(native.Step(source,input),0);
      t::mapped::NodalStiffness packet;
      ASSERT_TRUE(t::mapped::PackStiffness(trial.diagnostics,packet));
      const double coefficients[]{native.output[98],native.output[99]};
      Scatter(trial.internal_force,trial.internal_couple,packet,native.output.data()+75,
          native.output.data()+84,coefficients);
      removed|=trial.removed_now;
      accepted=trial.proposed_history;
    }
    if (near_failure) EXPECT_TRUE(removed);
  }
}
} // namespace qt_mapped_test
