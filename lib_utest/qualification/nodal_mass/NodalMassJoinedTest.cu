#include "NodalMassTestSupport.h"
#include "lib_src/elements/ShellBatchPublication.h"
#include "lib_utest/qualification/nodal/NodalTemporalFixture.h"

namespace nodal_mass_test {
namespace temporal=tl_test::nodal_temporal;
namespace q=fe::qeph;
namespace t=fe::t3;
using NodalMassCuda=temporal::NodalTemporalCuda;
struct Joined {
  fe::ShellBatchBinding shells=Shells();
  SpringInput input{shells};
  spring::Model connectors=Connectors(input,shells.node_count());
  fe::NodalMassBinding mass;
  temporal::Initial initial;
  fe::FENodalState owner;
  q::QephBatch qb;
  t::T3Batch tb;
  bool Initialize(bool owner_combined,bool batches_combined) {
    EXPECT_TRUE(mass.Initialize(shells,connectors));
    initial.n=shells.node_count(); initial.h=0x1p-20;
    for(std::size_t n=0;n<initial.n;++n) {
      const auto& x=shells.nodes()[n].position;
      initial.x[3*n]=x.x; initial.x[3*n+1]=x.y; initial.x[3*n+2]=x.z;
      initial.inverse[n]=1./(owner_combined?mass.nodes()[n].coefficients.mass:shells.nodes()[n].native.mass);
      initial.inverse_inertia[n]=1./(owner_combined?mass.nodes()[n].coefficients.isotropic_inertia:shells.nodes()[n].native.isotropic_inertia);
    }
    const auto state=initial.Initialize(owner);
    EXPECT_EQ(state.status,fe::NodalStatus::Ok); if(state.status!=fe::NodalStatus::Ok)return false;
    q::QephBatchConfig qc; qc.owner=owner.accepted(); qc.element_count=1;
    qc.configuration_id=7; qc.qualification_id=8; qc.usage=q::BatchUsage::CoupledForces;
    t::T3BatchConfig tc; tc.owner=qc.owner; tc.element_count=1;
    tc.configuration_id=7; tc.qualification_id=8; tc.usage=t::BatchUsage::CoupledForces;
    const auto qr=batches_combined?qb.InitializeJoined(qc,shells,mass):qb.InitializeJoined(qc,shells);
    const auto tr=batches_combined?tb.InitializeJoined(tc,shells,mass):tb.InitializeJoined(tc,shells);
    EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message;
    EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
    return qr.status==q::BatchStatus::Success&&tr.status==t::BatchStatus::Success;
  }
};
TEST_F(NodalMassCuda, BothFamiliesBindCombinedActualCoefficientsWithoutPublishingMissingConnector) {
  Joined rig; ASSERT_TRUE(rig.Initialize(true,true));
  temporal::Snapshot before,after; ASSERT_TRUE(temporal::Read(rig.owner,before));
  for(unsigned retry=0;retry<2;++retry) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(rig.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    EXPECT_EQ(rig.qb.AssembleAccepted(rig.owner,view).status,q::BatchStatus::Success);
    EXPECT_EQ(rig.tb.AssembleAccepted(rig.owner,view).status,t::BatchStatus::Success);
    rig.owner.Discard(); rig.qb.DiscardTrial(); rig.tb.DiscardTrial();
    fe::ShellBatchPublication publication;
    EXPECT_EQ(publication.Initialize(rig.owner,rig.qb,rig.tb).status,fe::ShellPublicationStatus::NotJoined);
    EXPECT_EQ(publication.allocations().device_bytes,0u);
    ASSERT_TRUE(temporal::Read(rig.owner,after)); temporal::SameState(before,after);
  }
}
TEST_F(NodalMassCuda, ShellOnlyAndCombinedCoefficientsCannotBeSubstitutedAtTheOwner) {
  for(bool combined_owner:{false,true}) {
    Joined rig; ASSERT_TRUE(rig.Initialize(combined_owner,!combined_owner));
    temporal::Snapshot before,after; ASSERT_TRUE(temporal::Read(rig.owner,before));
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(rig.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    EXPECT_EQ(rig.qb.AssembleAccepted(rig.owner,view).status,q::BatchStatus::InvalidMass);
    // A rejected contributor makes the entire assembly sticky. Check the
    // second family's independent coefficient admission on a fresh attempt.
    rig.owner.Discard(); rig.qb.DiscardTrial(); rig.tb.DiscardTrial();
    ASSERT_EQ(rig.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    EXPECT_EQ(rig.tb.AssembleAccepted(rig.owner,view).status,t::BatchStatus::InvalidMass);
    rig.owner.Discard(); rig.qb.DiscardTrial(); rig.tb.DiscardTrial();
    ASSERT_TRUE(temporal::Read(rig.owner,after)); temporal::SameState(before,after);
  }
}
TEST_F(NodalMassCuda, CompleteSourceMismatchRejectsBeforeDeviceAllocationAndCanRetry) {
  Joined rig; ASSERT_TRUE(rig.Initialize(true,true));
  auto changed=shell_binding_test::Edge(); changed.qeph.young_modulus*=2.;
  fe::ShellBatchBinding wrong;
  ASSERT_EQ(wrong.Initialize(changed).status,fe::ShellBindingStatus::Success);
  q::QephBatch qb; q::QephBatchConfig qc; qc.owner=rig.owner.accepted();
  qc.element_count=1; qc.configuration_id=7; qc.qualification_id=8;
  qc.usage=q::BatchUsage::CoupledForces;
  EXPECT_EQ(qb.InitializeJoined(qc,wrong,rig.mass).status,q::BatchStatus::InvalidInput);
  EXPECT_EQ(qb.allocations().device_bytes,0u);
  EXPECT_EQ(qb.InitializeJoined(qc,rig.shells,rig.mass).status,q::BatchStatus::Success);
}
} // namespace nodal_mass_test
