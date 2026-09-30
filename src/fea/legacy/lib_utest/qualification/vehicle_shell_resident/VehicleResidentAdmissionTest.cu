#include "VehicleResidentFixture.h"
namespace vehicle_resident_test {
TEST_F(CudaTest, VehicleByteAndLegacyProfileFailuresPrecedeBorrowedElementReads) {
  auto config=[](auto c){c.element_count=328344;c.owner.node_count=359785;
    c.storage_limits=fe::ShellResidentLimits::Vehicle();c.max_device_bytes=fe::MaxVehicleShellResidentDeviceBytes;return c;};
  const auto qc=config(q::QephBatchConfig{});const auto tc=config(t::T3BatchConfig{});
  q::QephBatch qeph;t::T3Batch t3;
  for(unsigned fault=0;fault<6;++fault){auto q=qc;auto t=tc;
    if(fault==0){q.storage_limits.profile=t.storage_limits.profile=fe::ShellResidentProfile::Legacy;}
    if(fault==1){q.storage_limits.max_parents=t.storage_limits.max_parents=524289;}
    if(fault==2){q.storage_limits.max_nodes=t.storage_limits.max_nodes=524289;}
    if(fault==3){q.max_device_bytes=t.max_device_bytes=1;}
    if(fault==4){q.storage_limits.max_host_bytes=t.storage_limits.max_host_bytes=1;}
    if(fault==5){q.max_device_bytes=t.max_device_bytes=fe::MaxVehicleShellResidentDeviceBytes+1;}
    EXPECT_EQ(qeph.Initialize(q,reinterpret_cast<const q::QephBatchElement*>(1)).status,q::BatchStatus::ResourceLimit)<<fault;
    EXPECT_EQ(t3.Initialize(t,reinterpret_cast<const t::T3BatchElement*>(1)).status,t::BatchStatus::ResourceLimit)<<fault;
    EXPECT_EQ(qeph.allocations().device_allocations,0u);EXPECT_EQ(t3.allocations().device_allocations,0u);
  }
}
TEST_F(CudaTest, VehiclePublicationRequiresCompleteBindingAndExplicitProfileAndPreservesSourceCopies) {
  Rig r(2049,73,4097);ASSERT_TRUE(r.Initialize(true));
  q::QephBatch incomplete;auto config=r.QConfig();--config.element_count;
  EXPECT_EQ(incomplete.InitializeJoined(config,r.binding,r.catalog).status,q::BatchStatus::InvalidInput);
  EXPECT_EQ(incomplete.allocations().device_allocations,0u);
  // Invalid limit preflight precedes the existing coordinator-claim check.
  fe::ShellBatchPublication other;auto limits=fe::ShellPublicationLimits::Vehicle();
  limits.profile=fe::ShellResidentProfile::Legacy;
  EXPECT_EQ(other.Initialize(r.owner,r.qeph,r.t3,limits).status,fe::ShellPublicationStatus::ResourceLimit);
  limits=fe::ShellPublicationLimits::Vehicle();limits.max_device_bytes=1;
  EXPECT_EQ(other.Initialize(r.owner,r.qeph,r.t3,limits).status,fe::ShellPublicationStatus::ResourceLimit);
  limits=fe::ShellPublicationLimits::Vehicle();limits.max_host_bytes=1;
  EXPECT_EQ(other.Initialize(r.owner,r.qeph,r.t3,limits).status,fe::ShellPublicationStatus::ResourceLimit);
  EXPECT_EQ(other.allocations().device_allocations,0u);
  Results base(r.nq,r.nt,true),next(r.nq,r.nt,true);ASSERT_TRUE(Accepted(r,base));
  // Mutable original inputs do not replace either immutable native producer.
  const auto mass=r.binding.totals().mass;const auto old=r.source.geometry.q.back().reference.young_modulus;
  r.source.geometry.q.back().reference.young_modulus=1;r.source.materials[0].young_pa=1;r.source.y[0]=1;
  EXPECT_EQ(r.binding.totals().mass,mass);EXPECT_EQ(r.binding.qeph_reference(r.nq-1).input.young_modulus,old);
  Prepared p(r.n);ASSERT_TRUE(Prepare(r,p,true));ASSERT_TRUE(Evaluate(r,p,next));
  ASSERT_NO_FATAL_FAILURE(CheckNativeTail(r,p,base,next));ASSERT_TRUE(Publish(r,p,next));
}
} // namespace vehicle_resident_test
