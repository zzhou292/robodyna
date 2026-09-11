// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"
#include "../tied_cin_runtime/OwnerFixture.h"

namespace type13_resident_test {
TEST_F(Type13ResidentCuda, ActualCinDestinationReceivesStiffnessWithZeroDependentInverses) {
  cin_runtime_test::Fixture cin_source;
  std::fill(cin_source.velocity.begin(), cin_source.velocity.end(), 0);
  const auto& domain = cin_source.source.domain;
  const auto a = cin_source.rows[0].secondary;
  // The two independent test patches have coincident secondary coordinates.
  // Use one actual dependent and a distinct master coordinate; do not invent
  // a zero-length beam or merge their different source identities.
  const auto b = cin_source.rows[1].masters[0];
  ASSERT_EQ(cin_source.inverse[a], 0);
  ASSERT_EQ(cin_source.inverse_j[a], 0);
  type13_test::Fixture material;
  auto property = material.Input();
  property.units = {1, 1, 1}; // Explicit synthetic SI property packet.
  t::ModelPropertyInput declaration{200, property};
  const auto orientation = cin_source.rows[0].masters[1];
  const std::size_t source_nodes[]{a, b, orientation};
  std::array<t::ModelNode, 3> nodes;
  for (unsigned local = 0; local < 3; ++local) {
    const auto global = source_nodes[local];
    const auto& node = domain.nodes()[global];
    nodes[local] = {node.source_id, global, node.position};
  }
  t::ModelConnection connection{100, 0, {0, 1, 2}};
  t::Model model;
  const auto model_report = model.Initialize({domain.source_instance_id(), {1, 1, 1}, nodes.data(),
      &declaration, &connection, nodes.size(), 1, 1, domain.node_count()});
  ASSERT_TRUE(model_report) << model_report.message << ", status=" << int(model_report.status)
      << ", kind=" << int(model_report.kind) << ", entry=" << model_report.entry
      << ", native_status=" << int(model_report.native_status);
  fe::Type13NodeContributions contributions;
  ASSERT_TRUE(contributions.Initialize(model, domain));
  fe::FENodalState owner;
  auto owner_config = cin_source.Config();
  owner_config.fixed_dt = Config(domain.node_count()).owner.fixed_dt;
  ASSERT_TRUE(Good(owner.Initialize(owner_config, cin_source.Kinematics(),
      cin_source.inverse.data(), cin_source.Dofs(), cin_source.Startup())));
  auto config = Config(domain.node_count());
  config.owner = owner.accepted();
  config.qualification_id = 871;
  t::Batch omitted_stiffness;
  ASSERT_TRUE(Good(omitted_stiffness.InitializeJoined(config, contributions)));
  fe::NodalTrialToken wrong_token;
  fe::NodalAssemblyView wrong_view;
  ASSERT_TRUE(Good(owner.BeginTrial(&wrong_token, &wrong_view)));
  EXPECT_EQ(omitted_stiffness.AssembleAccepted(owner, wrong_token, wrong_view).status,
            t::BatchStatus::InvalidInput);
  EXPECT_EQ(owner.accepted().epoch, 0u);
  config.assembly = t::BatchAssembly::CinNativeStiffness;
  t::Batch batch;
  ASSERT_TRUE(Good(batch.InitializeJoined(config, contributions)));
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  fe::NodalCinAssemblyView cin;
  ASSERT_NO_FATAL_FAILURE(cin_runtime_test::Fill(owner, cin_source, token, view, cin));
  ASSERT_TRUE(Good(batch.AssembleAccepted(owner, token, view)));
  std::vector<t::Evaluation> accepted(1);
  t::BatchDiagnostics initial;
  ASSERT_TRUE(Good(batch.CopyAcceptedResults(owner.accepted(), accepted.data(), 1, &initial)));
  t::NativeEndpointKinematics native_initial_packet[2];
  native_initial_packet[0].position = nodes[0].position_native;
  native_initial_packet[1].position = nodes[1].position_native;
  const auto native_initial = type13_recurrence_test::NativeEvaluate(*model.property(0),
      model.startup(0)->reference, Virgin(model.startup(0)->reference), native_initial_packet, 0, true);
  Agreement(accepted[0], native_initial);
  const auto stiffness = NativeStiffness(*model.property(0), native_initial);
  std::vector<double> translation(domain.node_count()), rotation(domain.node_count());
  ASSERT_EQ(cudaMemcpyAsync(translation.data(), cin.translational_stiffness,
      translation.size()*sizeof(double), cudaMemcpyDeviceToHost, view.stream), cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(rotation.data(), cin.rotational_stiffness,
      rotation.size()*sizeof(double), cudaMemcpyDeviceToHost, view.stream), cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(view.stream), cudaSuccess);
  for (std::size_t i = 0; i < domain.node_count(); ++i) {
    const bool endpoint = i == a || i == b;
    EXPECT_DOUBLE_EQ(translation[i], cin_source.stif[i] + (endpoint ? stiffness[0] : 0));
    EXPECT_DOUBLE_EQ(rotation[i], cin_source.stifr[i] + (endpoint ? stiffness[2] : 0));
  }
  ASSERT_TRUE(Good(owner.SealAssembly(token)));
  ASSERT_TRUE(Good(fe::AdvanceStaggeredCin(owner, token, cin_runtime_test::Admission(view))));
  fe::NodalPreparedView prepared;
  ASSERT_TRUE(Good(owner.BorrowPrepared(token, &prepared)));
  t::BatchDiagnostics diagnostics;
  ASSERT_TRUE(Good(batch.EvaluateCandidate(owner, token, prepared, &diagnostics)));
  std::vector<t::Evaluation> trial(1);
  ASSERT_TRUE(Good(batch.CopyPreparedResults(diagnostics, trial.data(), 1)));
  Snapshot fields(domain.node_count());
  fe::NodalPreparedView copied;
  ASSERT_TRUE(Good(owner.CopyPrepared(token, fields.Buffer(), &copied)));
  t::NativeEndpointKinematics packet[2];
  const std::size_t ends[2]{a, b};
  for (unsigned local = 0; local < 2; ++local) {
    const auto n = ends[local];
    ASSERT_TRUE(detail::FromSI({1, 1, 1},
        {fields.x[3*n], fields.x[3*n+1], fields.x[3*n+2]},
        {fields.v[3*n], fields.v[3*n+1], fields.v[3*n+2]},
        {fields.w[3*n], fields.w[3*n+1], fields.w[3*n+2]}, packet[local]));
  }
  const auto native = type13_recurrence_test::NativeEvaluate(*model.property(0),
      model.startup(0)->reference, native_initial.native_history, packet, config.owner.fixed_dt);
  Agreement(trial[0], native);
  ASSERT_TRUE(Good(t::BatchQualificationPeer::Commit(batch, owner, token, prepared, diagnostics)));
  EXPECT_EQ(owner.accepted().epoch, 1u);
  ASSERT_TRUE(Good(batch.CopyAcceptedResults(owner.accepted(), accepted.data(), 1, &initial)));
  EXPECT_EQ(initial.phase, t::BatchPhase::Accepted);
  Exact(accepted[0], trial[0]);
}
} // namespace type13_resident_test
