// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"
#include "../type13_model/OriginalSource.h"

namespace type13_resident_test {
TEST_F(Type13ResidentCuda, Original4442LoadedOwnerTrajectoryMatchesEveryNativeHistory) {
  type13_model_test::Source input;
  Rig rig;
  ASSERT_TRUE(rig.Initialize(input.Input(), true));
  ASSERT_EQ(rig.source.model.connection_count(), 4442u);
  ASSERT_EQ(rig.owner.accepted().node_count, 7493u);
  RecordProperty("device_bytes", std::to_string(rig.batch.allocations().device_bytes));
  RecordProperty("startup_host_bytes", std::to_string(rig.batch.startup_host_bytes()));
  double maximum_force = 0;
  for (unsigned step = 0; step < 8; ++step) {
    SCOPED_TRACE(step);
    fe::NodalTrialToken token;
    fe::NodalAssemblyView view;
    fe::NodalPreparedView prepared;
    ASSERT_TRUE(rig.Begin(token, view, 1e9));
    rig.CompareAssembly(view);
    ASSERT_TRUE(rig.Prepare(token, view, prepared));
    t::BatchDiagnostics diagnostics;
    ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner, token, prepared, &diagnostics)));
    std::vector<t::Evaluation> values(4442), native_next;
    ASSERT_TRUE(Good(rig.batch.CopyPreparedResults(diagnostics, values.data(), values.size())));
    rig.ComparePrepared(token, prepared, values, native_next);
    for (const auto& value : values) {
      maximum_force = std::max(maximum_force, tl::math::fixed3::Norm(value.local_force_N));
    }
    ASSERT_TRUE(Good(t::BatchQualificationPeer::Commit(rig.batch, rig.owner, token, prepared, diagnostics)));
    rig.native = std::move(native_next);
  }
  EXPECT_GT(maximum_force, 1e-3);
  EXPECT_EQ(rig.owner.accepted().epoch, 8u);
  EXPECT_EQ(rig.batch.allocations().device_allocations, 1u);
}
} // namespace type13_resident_test
