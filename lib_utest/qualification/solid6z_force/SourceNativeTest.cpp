// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include "Compare.h"
#include "../solid6z_reference/SourceFixture.h"
namespace solid6z_force_test {
TEST(Solid6zForceSource, All195MappedOriginalWedgesAdvanceIndependentNativeCaller) {
  unsigned count = 0;
  for (unsigned row = 0; row < solid6z_test::SourceCount; ++row) {
    s::ReferenceInput input;
    if (!solid6z_test::Source(row,input)) continue;
    SCOPED_TRACE(input.source_element_id);
    const auto reference = Reference(input);
    const auto material = Material(input.density_kg_m3);
    auto accepted = Initial(reference,material);
    NativeHistory native;
    ASSERT_TRUE(native.Initialize(input,material));
    const auto origin = input.position_m[0];
    const double length = reference.geometry().characteristic_length_m;
    auto position = [&](unsigned n,double phase) {
      const auto x = input.position_m[n];
      const double a = x.x-origin.x, b = x.y-origin.y, c = x.z-origin.z;
      const double p[6]{1,-.7,.3,-.8,.6,-.4};
      return s::Vec3{origin.x+(1+.05*phase)*a+.01*phase*b+.002*length*p[n]*phase,
          origin.y+(1-.03*phase)*b+.02*phase*c-.001*length*p[n]*phase,
          origin.z+(1+.02*phase)*c+.01*phase*a+.002*length*p[n]*phase};
    };
    constexpr double phase[]{0,.2,.4,.3,.1,.2};
    for (unsigned step = 0; step < 5; ++step) {
      s::PrescribedInterval interval;
      interval.dt_s = 1e-6;
      interval.base_time_s = accepted.stamp().time_s;
      interval.sample_index = step;
      for (unsigned n = 0; n < 6; ++n) {
        const auto before = position(n,phase[step]);
        const auto after = position(n,phase[step+1]);
        interval.position_endpoint_m[n] = after;
        interval.velocity_midpoint_m_s[n] = {(after.x-before.x)/interval.dt_s,
            (after.y-before.y)/interval.dt_s,(after.z-before.z)/interval.dt_s};
      }
      const auto expected = native.Evaluate(interval);
      s::ForceTrial result;
      ASSERT_EQ(s::EvaluateForce(reference,accepted,interval,material,{},result),s::Status::Success) << step;
      ASSERT_TRUE(Agree(Pack(result),expected)) << step;
      for (unsigned n = 0; n < 6; ++n)
        ASSERT_EQ(result.proposed_history.reference().input().source_node_id[n],input.source_node_id[n]);
      accepted = result.proposed_history;
      native.Accept(expected);
    }
    ++count;
  }
  EXPECT_EQ(count,195u);
  RecordProperty("mapped_original_wedges",count);
  RecordProperty("native_force_packets",5*count);
}
} // namespace solid6z_force_test
