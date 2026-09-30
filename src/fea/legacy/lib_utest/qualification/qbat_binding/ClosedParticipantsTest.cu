#include "Fixture.h"
#include "../nodal_mass/NodalMassTestSupport.h"
#include "lib_src/elements/qeph/QephBatch.h"
#include "lib_src/elements/t3/T3Batch.h"
#include <string>

namespace qbat_binding_test {
TEST(QbatBindingClosedParticipants, OldQephAndT3JoinedEntriesRejectBeforeOwnerOrAllocation) {
  Fixture f;
  Binding binding;
  ASSERT_EQ(binding.InitializeFormulations(f.Input()).status,Status::Success);
  const nodal_mass_test::SpringInput fixture(binding);
  const auto connectors=nodal_mass_test::Connectors(fixture,binding.node_count());
  fe::NodalMassBinding mass;
  ASSERT_TRUE(mass.Initialize(binding,connectors));
  fe::qeph::QephBatch q;
  fe::t3::T3Batch t;
  const fe::qeph::QephBatchConfig qc;
  const fe::t3::T3BatchConfig tc;
  const char* required="QBAT requires a complete formulation publication participant";
  const auto q0=q.InitializeJoined(qc,binding);
  const auto t0=t.InitializeJoined(tc,binding);
  EXPECT_EQ(q0.status,fe::qeph::BatchStatus::InvalidInput);
  EXPECT_EQ(t0.status,fe::t3::BatchStatus::InvalidInput);
  EXPECT_EQ(std::string(q0.message),required);
  EXPECT_EQ(std::string(t0.message),required);
  const auto q1=q.InitializeJoined(qc,binding,mass);
  const auto t1=t.InitializeJoined(tc,binding,mass);
  EXPECT_EQ(q1.status,fe::qeph::BatchStatus::InvalidInput);
  EXPECT_EQ(t1.status,fe::t3::BatchStatus::InvalidInput);
  EXPECT_EQ(std::string(q1.message),required);
  EXPECT_EQ(std::string(t1.message),required);
  EXPECT_EQ(q.allocations().device_allocations,0u);
  EXPECT_EQ(t.allocations().device_allocations,0u);
  // No owner is created and no device operation is necessary. This executable
  // qualifies the compiled CUDA participant admission seam, not QBAT dynamics.
}
} // namespace qbat_binding_test
