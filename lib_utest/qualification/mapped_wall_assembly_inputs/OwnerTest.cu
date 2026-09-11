// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_utest/qualification/physical_mesh_wall/Fixture.h"
#include <limits>
namespace physical_wall_test {
TEST(MappedWallAssemblyInputsOwner, ActualMassAndStartupGeometryRejectionPreserveHistoryThenRetry) {
  p::Rig rig(true);
  ASSERT_TRUE(rig.Initialize());
  Geometry geometry(rig.fixture);
  c::NodalWallMappedContact contact;
  ASSERT_TRUE(Good(contact.Initialize(Config(rig), geometry.Wall(), geometry.weights,
      Source(rig), rig.owner, geometry.Motion())));
  const auto allocation = contact.allocations();
  const auto stamp = rig.owner.accepted();
  const auto node = geometry.weights.node(geometry.weights.node_count()-1).node;
  p::Snapshot before, after;
  ASSERT_TRUE(rig.Read(before));
  for (bool geometry_fault : {false, true}) {
    SCOPED_TRACE(geometry_fault);
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(rig.Begin(token, assembly));
    // Deliberate test-only mutation of the actual immutable borrowed source.
    // Restore it before asking the owner to expose accepted state again.
    auto* target = const_cast<double*>(geometry_fault
        ? assembly.accepted.position_xyz+3*node
        : assembly.mass.inverse_mass+node);
    double original = 0;
    ASSERT_EQ(cudaMemcpyAsync(&original, target, sizeof(original), cudaMemcpyDeviceToHost,
        assembly.stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(assembly.stream), cudaSuccess);
    const double broken = geometry_fault ? ::nextafter(original, INFINITY)
        : std::numeric_limits<double>::quiet_NaN();
    ASSERT_EQ(cudaMemcpyAsync(target, &broken, sizeof(broken), cudaMemcpyHostToDevice,
        assembly.stream), cudaSuccess);
    c::NodalWallMappedDiagnostics output;
    output.accepted_active_parents = 913;
    const auto report = contact.AssembleAccepted(rig.owner, token, assembly, &output);
    EXPECT_EQ(report.status, geometry_fault ? c::NodalWallDeviceStatus::GeometryFailure
                                           : c::NodalWallDeviceStatus::InvalidMass);
    EXPECT_EQ(report.node, node);
    EXPECT_EQ(output.accepted_active_parents, 913u);
    ASSERT_EQ(cudaMemcpyAsync(target, &original, sizeof(original), cudaMemcpyHostToDevice,
        assembly.stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(assembly.stream), cudaSuccess);
    rig.owner.Discard();
    rig.publication.DiscardTrial();
    contact.DiscardTrial();
    ASSERT_TRUE(rig.Read(after));
    p::Exact(before, after);
    EXPECT_TRUE(fe::trial_identity::SameStamp(stamp, rig.owner.accepted()));
    ASSERT_TRUE(rig.Begin(token, assembly));
    ASSERT_TRUE(Good(contact.AssembleAccepted(rig.owner, token, assembly, &output)));
    rig.owner.Discard();
    rig.publication.DiscardTrial();
    contact.DiscardTrial();
    EXPECT_EQ(contact.allocations().device_bytes, allocation.device_bytes);
    EXPECT_EQ(contact.allocations().device_allocations, allocation.device_allocations);
  }
}
} // namespace physical_wall_test
