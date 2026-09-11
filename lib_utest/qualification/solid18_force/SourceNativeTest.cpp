// SPDX-License-Identifier: AGPL-3.0-or-later
#include "PacketValues.h"
#include "lib_utest/qualification/solid18_reference/SourceFixture.h"

namespace solid18_force_test {
TEST(Solid18ForceSource, OriginalAdhesiveRepresentativeCellsNativeCurrentGeometryAndForce) {
  // First/final source identity plus the independently diagnosed distorted cell.
  // This is a force gate on selected original rows, not a 908-element assembly.
  const unsigned rows[] = {0,89,solid18_test::SourceCount-1};
  const auto material = Material();
  for (unsigned row : rows) {
    const auto input = solid18_test::Source(row);
    SCOPED_TRACE(input.source_element_id);
    s::Reference reference;
    ASSERT_EQ(s::InitializeReference(input,reference),s::Status::Success);
    auto native = NativeInitial(input);
    s::History accepted;
    ASSERT_EQ(s::InitializeHistory(reference,material,accepted),s::Status::Success);
    for (unsigned n = 0; n < 8; ++n) {
      ASSERT_EQ(reference.source_slot(n),native.source_slot[n]);
      ASSERT_EQ(reference.input().source_node_id[n],input.source_node_id[n]);
    }
    const auto origin = input.position_m[0];
    constexpr double dt = 1.0/1048576;
    for (unsigned step = 0; step < 32; ++step) {
      SCOPED_TRACE(step);
      s::PrescribedInterval interval;
      interval.base_time_s = step*dt;
      interval.dt_s = dt;
      interval.sample_index = step+1;
      const double time = (step+1)*dt;
      for (unsigned n = 0; n < 8; ++n) {
        const auto p = input.position_m[n];
        const s::Vec3 d{p.x-origin.x,p.y-origin.y,p.z-origin.z};
        const s::Vec3 velocity{100*d.x+15*d.y,-35*d.y+9*d.z,-20*d.z+11*d.x};
        interval.position_endpoint_m[n] = {p.x+time*velocity.x,p.y+time*velocity.y,p.z+time*velocity.z};
        interval.velocity_midpoint_m_s[n] = velocity;
      }
      const auto expected = Native(material,native,interval);
      ASSERT_EQ(expected.status,0);
      s::ForceTrial actual;
      ASSERT_EQ(s::EvaluateForce(reference,accepted,interval,material,actual),s::Status::Success);
      ASSERT_TRUE(Agree(actual,expected));
      native = expected.next;
      accepted = actual.proposed_history;
    }
  }
  RecordProperty("source_force_cells",3);
  RecordProperty("source_force_steps_each",32);
}
} // namespace solid18_force_test
