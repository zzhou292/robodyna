// SPDX-License-Identifier: MIT
#include "ResidentFixture.h"
#include "../nodal_mass/NodalMassTestSupport.h"

namespace qbat_resident_test {
TEST(QbatResidentCuda, QbatAndConnectorUseCombinedCoefficientsAndOnePublication) {
  MidlayerSource source(false);
  nodal_mass_test::SpringInput input(source.binding);
  const auto model=nodal_mass_test::Connectors(input,source.binding.node_count());
  fe::NodalMassBinding mass;
  ASSERT_EQ(mass.Initialize(source.binding,model).status,fe::NodalMassStatus::Success);
  auto scope=source.Scope();
  scope.mass=&mass;
  Rig rig;
  ASSERT_TRUE(rig.Initialize(scope,true,true,0x1p-26,&model));
  Fields before;
  std::vector<qb::BatchResult> old;
  fe::ShellBatchDiagnostics initial;
  ASSERT_TRUE(rig.Accepted(before,old,initial));
  ASSERT_TRUE(initial.has_connector);
  const auto allocations=rig.qbat.allocations();
  for(unsigned step=0;step<8;++step) {
    Prepared prepared;
    ASSERT_TRUE(rig.Prepare(prepared,100));
    EXPECT_TRUE(prepared.diagnostics.has_connector);
    EXPECT_TRUE(prepared.diagnostics.connector.accepted_force_assembled);
    long double translation=0,rotation=0,connector_translation=0,connector_rotation=0;
    for(std::size_t node=0;node<mass.node_count();++node) {
      const auto& coefficients=mass.nodes()[node].coefficients;
      long double speed=0,spin=0;
      for(unsigned axis=0;axis<3;++axis) {
        const long double v=prepared.endpoint.v[3*node+axis];
        const long double w=prepared.endpoint.omega[3*node+axis];
        speed+=v*v;
        spin+=w*w;
      }
      translation+=.5L*coefficients.mass*speed;
      rotation+=.5L*coefficients.isotropic_inertia*spin;
      connector_translation+=.5L*coefficients.connector_mass*speed;
      connector_rotation+=.5L*coefficients.connector_inertia*spin;
    }
    nodal_mass_test::Near(prepared.diagnostics.kinetic.translation,translation);
    nodal_mass_test::Near(prepared.diagnostics.kinetic.rotation,rotation);
    nodal_mass_test::Near(prepared.diagnostics.kinetic.connector_translation,connector_translation);
    nodal_mass_test::Near(prepared.diagnostics.kinetic.connector_rotation,connector_rotation);
    if(step==0) {
      fe::ShellBatchDiagnostics output;
      const auto bytes=Bytes(output);
      const fe::ShellFormulationCandidates missing{nullptr,nullptr,&prepared.diagnostics.qbat,nullptr};
      EXPECT_EQ(rig.publication.PrepareFormulations(rig.owner,prepared.token,missing,&output).status,
          fe::ShellPublicationStatus::NotJoined);
      EXPECT_EQ(Bytes(output),bytes);
      Fields still;
      std::vector<qb::BatchResult> unchanged;
      fe::ShellBatchDiagnostics diagnostics;
      ASSERT_TRUE(rig.Accepted(still,unchanged,diagnostics));
      SameFields(before,still);
      Exact(old,unchanged);
      Prepared retry;
      ASSERT_TRUE(rig.Prepare(retry,100));
      Exact(prepared.qbat,retry.qbat);
      prepared=std::move(retry);
    }
    ASSERT_TRUE(rig.Commit(prepared));
    fe::type25::BatchDiagnostics connector;
    ASSERT_EQ(rig.connector.CopyAcceptedDiagnostics(rig.owner.accepted(),&connector).status,
        fe::type25::BatchStatus::Success);
    EXPECT_EQ(connector.epoch,rig.owner.accepted().epoch);
    EXPECT_EQ(connector.attempt,prepared.view.attempt);
    EXPECT_EQ(rig.qbat.allocations().device_bytes,allocations.device_bytes);
  }
}
} // namespace qbat_resident_test
