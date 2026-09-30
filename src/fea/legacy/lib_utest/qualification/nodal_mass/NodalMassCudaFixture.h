#pragma once
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
  bool Initialize(bool owner_combined,bool batches_combined,tl::math::Vec3 velocity={},double dt=0x1p-20) {
    EXPECT_TRUE(mass.Initialize(shells,connectors));
    initial.n=shells.node_count(); initial.h=dt;
    for(std::size_t n=0;n<initial.n;++n) {
      const auto& x=shells.nodes()[n].position;
      initial.x[3*n]=x.x; initial.x[3*n+1]=x.y; initial.x[3*n+2]=x.z;
      initial.v[3*n]=velocity.x; initial.v[3*n+1]=velocity.y; initial.v[3*n+2]=velocity.z;
      initial.inverse[n]=1./(owner_combined?mass.nodes()[n].coefficients.mass:shells.nodes()[n].native.mass);
      initial.inverse_inertia[n]=1./(owner_combined?mass.nodes()[n].coefficients.isotropic_inertia:shells.nodes()[n].native.isotropic_inertia);
    }
    const auto state=initial.Initialize(owner);
    EXPECT_EQ(state.status,fe::NodalStatus::Ok); if(state.status!=fe::NodalStatus::Ok)return false;
    q::QephBatchConfig qc; qc.owner=owner.accepted(); qc.element_count=1;
    qc.configuration_id=7; qc.qualification_id=8; qc.usage=q::BatchUsage::CoupledForces;
    t::T3BatchConfig tc; tc.owner=qc.owner; tc.element_count=1;
    tc.configuration_id=7; tc.qualification_id=8; tc.usage=t::BatchUsage::CoupledForces;
    if(velocity.x||velocity.y||velocity.z) {
      qc.startup={fe::ShellBatchStartupKind::ReferenceUniformTranslation,velocity};
      tc.startup=qc.startup;
    }
    const auto qr=batches_combined?qb.InitializeJoined(qc,shells,mass):qb.InitializeJoined(qc,shells);
    const auto tr=batches_combined?tb.InitializeJoined(tc,shells,mass):tb.InitializeJoined(tc,shells);
    EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message;
    EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
    return qr.status==q::BatchStatus::Success&&tr.status==t::BatchStatus::Success;
  }
};
} // namespace nodal_mass_test
