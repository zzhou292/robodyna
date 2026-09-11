#include "Fixture.h"
#include "lib_src/constraints/NodalRigidGroupModel.h"

namespace cin_runtime_test {
TEST(CinRuntimeStartup, ExactSourceAndCoincidentWitnessesRetainIndependentMechanicalIdentity) {
  Fixture f;
  auto input = f.Startup();
  f.witnesses.back().source_element_id = 987654; // Independent qualified coincident shell witness.
  fea::nodal_detail::CinLayout layout;
  ASSERT_EQ(fea::nodal_detail::ForecastCinStorage(input, f.Config(), layout).status, fea::NodalStatus::Ok);
  std::unique_ptr<fea::nodal_detail::CinStorage> output;
  ASSERT_EQ(fea::nodal_detail::PrepareCinStorage(input, f.Config(), f.Kinematics(), f.inverse.data(),
      f.Dofs(), nullptr, layout, output).status, fea::NodalStatus::Ok);
  EXPECT_EQ(output->witnesses.back().source_element_id, 987654u);
  EXPECT_EQ(output->source.rows().data[1].master_source.element_id, 401u);
  const auto* prior = output.get();
  f.witnesses.back().nodes[0] = f.rows.back().secondary;
  EXPECT_EQ(fea::nodal_detail::PrepareCinStorage(input, f.Config(), f.Kinematics(), f.inverse.data(),
      f.Dofs(), nullptr, layout, output).status, fea::NodalStatus::InvalidInput);
  EXPECT_EQ(output.get(), prior);
  f.witnesses.back().nodes[0] = f.rows.back().masters[0];
  f.witnesses.back().source_element_id = f.witnesses.front().source_element_id;
  EXPECT_EQ(fea::nodal_detail::PrepareCinStorage(input, f.Config(), f.Kinematics(), f.inverse.data(),
      f.Dofs(), nullptr, layout, output).status, fea::NodalStatus::InvalidInput);
  EXPECT_EQ(output.get(), prior);
  f.witnesses.back().source_element_id = 987654;
  ASSERT_EQ(fea::nodal_detail::PrepareCinStorage(input, f.Config(), f.Kinematics(), f.inverse.data(),
      f.Dofs(), nullptr, layout, output).status, fea::NodalStatus::Ok);
}
TEST(CinRuntimeStartup, CountsBeforeBorrowedInputAndLateScalarFailurePreserveOutput) {
  Fixture f;
  auto input = f.Startup();
  fea::nodal_detail::CinLayout layout;
  input.witness_count = input.limits.max_witnesses+1;
  input.witnesses = reinterpret_cast<const cin::ActiveWitness*>(16);
  EXPECT_EQ(fea::nodal_detail::ForecastCinStorage(input, f.Config(), layout).status, fea::NodalStatus::ResourceLimit);
  input = f.Startup();
  ASSERT_EQ(fea::nodal_detail::ForecastCinStorage(input, f.Config(), layout).status, fea::NodalStatus::Ok);
  std::unique_ptr<fea::nodal_detail::CinStorage> output;
  auto late = f.mass.size()-1;
  while (f.dependent[late]) --late;
  const auto original = f.mass[late];
  f.mass[late] *= 1.01;
  EXPECT_EQ(fea::nodal_detail::PrepareCinStorage(input, f.Config(), f.Kinematics(), f.inverse.data(),
      f.Dofs(), nullptr, layout, output).status, fea::NodalStatus::InvalidInput);
  EXPECT_FALSE(output);
  f.mass[late] = original;
  ASSERT_EQ(fea::nodal_detail::PrepareCinStorage(input, f.Config(), f.Kinematics(), f.inverse.data(),
      f.Dofs(), nullptr, layout, output).status, fea::NodalStatus::Ok);
}
TEST(CinRuntimeStartup, OriginalSignedZeroCoordinateIdentitySurvivesTheOwnerBoundary) {
  Fixture f;
  const auto node = f.rows.front().masters[0];
  f.source.nodes[node].position.z = -0.;
  f.source.declarations.front().reference_positions[1].z = -0.;
  fea::NodalNodeDomain domain;
  ASSERT_TRUE(domain.Initialize({73, f.source.nodes.data(), f.source.nodes.size()}));
  ASSERT_TRUE(tied::PrepareCinAttachments(f.source.post, domain, f.source.Input(), &f.model));
  f.x[3*node+2] = -0.;
  const auto input = f.Startup();
  fea::nodal_detail::CinLayout layout;
  ASSERT_EQ(fea::nodal_detail::ForecastCinStorage(input, f.Config(), layout).status, fea::NodalStatus::Ok);
  std::unique_ptr<fea::nodal_detail::CinStorage> output;
  ASSERT_EQ(fea::nodal_detail::PrepareCinStorage(input, f.Config(), f.Kinematics(), f.inverse.data(),
      f.Dofs(), nullptr, layout, output).status, fea::NodalStatus::Ok);
  const auto* prior = output.get();
  f.x[3*node+2] = 0.;
  EXPECT_EQ(fea::nodal_detail::PrepareCinStorage(input, f.Config(), f.Kinematics(), f.inverse.data(),
      f.Dofs(), nullptr, layout, output).status, fea::NodalStatus::InvalidInput);
  EXPECT_EQ(output.get(), prior);
}
TEST(CinRuntimeStartup, ZeroDependentInertiaIsLiteralAndHasNoConventionalInverse) {
  Fixture f;
  const auto node = f.rows.back().secondary;
  f.inertia[node] = 0;
  const auto input = f.Startup();
  fea::nodal_detail::CinLayout layout;
  ASSERT_EQ(fea::nodal_detail::ForecastCinStorage(input, f.Config(), layout).status, fea::NodalStatus::Ok);
  std::unique_ptr<fea::nodal_detail::CinStorage> output;
  ASSERT_EQ(fea::nodal_detail::PrepareCinStorage(input, f.Config(), f.Kinematics(), f.inverse.data(),
      f.Dofs(), nullptr, layout, output).status, fea::NodalStatus::Ok);
  std::vector<double> tail(layout.state_values, 0);
  output->InitializeState(tail.data(), input, f.inverse.data(), f.Dofs());
  const auto n = f.mass.size();
  EXPECT_EQ(tail[n+node], 0);
  EXPECT_EQ(tail[2*n+node], 0);
  EXPECT_EQ(tail[3*n+node], 0);
  EXPECT_EQ(tail[4*n+1], f.mass[node]);
  EXPECT_EQ(tail[4*n+f.rows.size()+1], 0);
  f.inverse_j[node] = 1;
  EXPECT_EQ(fea::nodal_detail::PrepareCinStorage(input, f.Config(), f.Kinematics(), f.inverse.data(),
      f.Dofs(), nullptr, layout, output).status, fea::NodalStatus::InvalidInput);
}
TEST(CinRuntimeStartup, ActualOwnerRigidMembershipIntersectionIsRejected) {
  Fixture f;
  std::vector<fea::NodalRigidGroupMember> members;
  for (unsigned slot = 0; slot < 3; ++slot) {
    const auto node = f.rows.front().masters[slot];
    const auto source = f.source.domain.nodes()[node];
    members.push_back({source.source_id, node, source.position, f.mass[node], f.inertia[node], f.inertia[node], 0});
  }
  const fea::NodalRigidGroupInput group{127, 337, members.data(), members.size()};
  fea::NodalRigidGroupModel groups;
  ASSERT_TRUE(groups.Initialize({73, f.mass.size(), &group, 1, {1000, .001}}));
  const auto input = f.Startup();
  fea::nodal_detail::CinLayout layout;
  ASSERT_EQ(fea::nodal_detail::ForecastCinStorage(input, f.Config(), layout).status, fea::NodalStatus::Ok);
  std::unique_ptr<fea::nodal_detail::CinStorage> output;
  const auto report = fea::nodal_detail::PrepareCinStorage(input, f.Config(), f.Kinematics(), f.inverse.data(),
      f.Dofs(), &groups, layout, output);
  EXPECT_EQ(report.status, fea::NodalStatus::InvalidInput);
  EXPECT_EQ(report.node, members.front().global_node);
  EXPECT_FALSE(output);
  ASSERT_EQ(fea::nodal_detail::PrepareCinStorage(input, f.Config(), f.Kinematics(), f.inverse.data(),
      f.Dofs(), nullptr, layout, output).status, fea::NodalStatus::Ok);
}
} // namespace cin_runtime_test
