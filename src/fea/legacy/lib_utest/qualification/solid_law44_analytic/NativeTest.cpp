// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
namespace law44_analytic_test {
TEST(SolidLaw44AnalyticNative, CompleteMfuncZeroReaderAndAcceptedTrajectory) {
  for (auto p : {Airbag(), Airbag(law::WorkingUnits::SI), Capped(.6), Capped(0)}) {
    law::History host, native;
    auto in = Motion(0); in.dt_s = 0;
    law::Result initial;
    ASSERT_EQ(law::Initialize(p, in, initial), law::Status::Ok);
    const auto n0 = Native(p, {}, in, true);
    law44_solid_test::Compare(initial, n0.result, p, {}, in);
    EXPECT_EQ(n0.prepared[25], 0); // No table CA-reset warning.
    EXPECT_EQ(n0.prepared[21], p.plastic_cap_strain);
    EXPECT_EQ(n0.prepared[4], p.failure_plastic_strain);
    EXPECT_EQ(n0.prepared[23], 0); // MFUNC0 reader leaves YLD_SCALE0.
    for (unsigned step = 0; step < 320; ++step) {
      in = Motion(step);
      law::Result result;
      ASSERT_EQ(law::Update(p, host, in, result), law::Status::Ok);
      const auto expected = Native(p, native, in);
      law44_solid_test::Compare(result, expected.result, p, native, in);
      host = result.history; native = expected.result.history;
    }
  }
}
TEST(SolidLaw44AnalyticNative, CapAndFailureNextafterNeighborsWithRetainedRate) {
  const auto p = Capped();
  for (double boundary : {0., p.plastic_cap_strain, p.failure_plastic_strain}) {
    for (double pla : {std::nextafter(boundary, 0.), boundary, std::nextafter(boundary, INFINITY)}) {
      law::History h; h.plastic_strain = pla; h.filtered_rate_per_s = 5;
      h.stress_pa[0] = 100e6; h.stress_pa[3] = 20e6;
      const auto in = Motion(21);
      law::Result result;
      ASSERT_EQ(law::Update(p, h, in, result), law::Status::Ok);
      law44_solid_test::Compare(result, Native(p, h, in).result, p, h, in);
    }
  }
}
} // namespace law44_analytic_test
