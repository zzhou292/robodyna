// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_utest/qualification/physical_mesh_wall/Fixture.h"
#include <limits>
namespace physical_wall_test {
TEST(MappedWallResponseOwner, ActualLateResponseFailurePreservesAllScatterDestinationsAndAcceptedHistoryThenRetry) {
  p::Rig rig(false);
  ASSERT_TRUE(rig.Initialize());
  Geometry geometry(rig.fixture);
  c::NodalWallMappedContact contact;
  auto config = Config(rig);
  fe::ShellMappedFootprint forecast;
  ASSERT_TRUE(Good(contact.Forecast(config, geometry.weights, Source(rig), forecast)));
  config.max_device_bytes = forecast.device_bytes-1;
  EXPECT_EQ(contact.Initialize(config, geometry.Wall(), geometry.weights, Source(rig), rig.owner,
      geometry.Motion()).status, c::NodalWallDeviceStatus::ResourceLimit);
  EXPECT_EQ(contact.allocations().device_bytes, 0u);
  config.max_device_bytes = forecast.device_bytes;
  ASSERT_TRUE(Good(contact.Initialize(config, geometry.Wall(), geometry.weights, Source(rig), rig.owner, geometry.Motion())));
  EXPECT_EQ(contact.allocations().device_bytes, forecast.device_bytes);
  const auto node = rig.fixture.domain.Find(14);
  ASSERT_EQ(rig.fixture.rigid.FindMember(node), nullptr);
  ASSERT_GT(rig.fixture.x[3*node], config.law.wall_x);
  const auto count = rig.fixture.domain.node_count();
  std::vector<double> before_scatter(8*count), after_scatter(8*count);
  p::Snapshot before, after;
  ASSERT_TRUE(rig.Read(before));
  for (bool overflow : {true, false}) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(rig.Begin(token, assembly));
    fe::NodalCinAssemblyView cin;
    ASSERT_TRUE(p::Good(rig.owner.BorrowCinAssembly(token, &cin)));
    double* channels[]{assembly.forces.force_x, assembly.forces.force_y, assembly.forces.force_z,
        assembly.forces.couple_x, assembly.forces.couple_y, assembly.forces.couple_z,
        cin.translational_stiffness, cin.rotational_stiffness};
    for (unsigned j = 0; j < 8; ++j)
      ASSERT_EQ(cudaMemcpyAsync(before_scatter.data()+j*count, channels[j], count*sizeof(double),
          cudaMemcpyDeviceToHost, assembly.stream), cudaSuccess);
    auto* inverse = const_cast<double*>(assembly.mass.inverse_mass)+node;
    double saved = 0;
    ASSERT_EQ(cudaMemcpyAsync(&saved, inverse, sizeof(saved), cudaMemcpyDeviceToHost, assembly.stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(assembly.stream), cudaSuccess);
    // Test-only fault after initial source admission: finite positive values
    // pass the mass validator and reach the response arithmetic/step check.
    const double broken = overflow ? std::numeric_limits<double>::max() : 1e24;
    ASSERT_EQ(cudaMemcpyAsync(inverse, &broken, sizeof(broken), cudaMemcpyHostToDevice, assembly.stream), cudaSuccess);
    c::NodalWallMappedDiagnostics output;
    output.accepted_active_parents = 731;
    const auto report = contact.AssembleAccepted(rig.owner, token, assembly, &output);
    EXPECT_EQ(report.status, overflow ? c::NodalWallDeviceStatus::NonFiniteArithmetic
                                    : c::NodalWallDeviceStatus::StepTooLarge);
    if (overflow) EXPECT_EQ(report.node, node);
    EXPECT_EQ(output.accepted_active_parents, 731u);
    for (unsigned j = 0; j < 8; ++j)
      ASSERT_EQ(cudaMemcpyAsync(after_scatter.data()+j*count, channels[j], count*sizeof(double),
          cudaMemcpyDeviceToHost, assembly.stream), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(inverse, &saved, sizeof(saved), cudaMemcpyHostToDevice, assembly.stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(assembly.stream), cudaSuccess);
    for (std::size_t i = 0; i < before_scatter.size(); ++i)
      EXPECT_EQ(p::Bits(before_scatter[i]), p::Bits(after_scatter[i])) << i;
    rig.owner.Discard(); rig.publication.DiscardTrial(); contact.DiscardTrial();
    ASSERT_TRUE(rig.Read(after)); p::Exact(before, after);
    ASSERT_TRUE(rig.Begin(token, assembly));
    ASSERT_TRUE(Good(contact.AssembleAccepted(rig.owner, token, assembly, &output)));
    EXPECT_GT(output.current_response_rate_upper, 0);
    rig.owner.Discard(); rig.publication.DiscardTrial(); contact.DiscardTrial();
    EXPECT_EQ(contact.allocations().device_bytes, forecast.device_bytes);
  }
}
} // namespace physical_wall_test
