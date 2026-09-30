// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaFixture.h"
#include "lib_src/elements/type25/Type25BatchArena.h"

namespace type25_batch_test {
TEST_F(Type25BatchCuda, MaximumActiveStorageAndStartupFieldsReachFinalConnection) {
  Rig rig;ASSERT_TRUE(rig.Initialize(1024,true));std::vector<spring::Evaluation> initial;spring::BatchDiagnostics diagnostics;
  ASSERT_TRUE(rig.Read(initial,diagnostics));ASSERT_EQ(initial.size(),1024u);EXPECT_EQ(diagnostics.active_count,1024u);
  EXPECT_EQ(diagnostics.epoch,0u);EXPECT_EQ(diagnostics.phase,spring::BatchPhase::Accepted);
  EXPECT_EQ(rig.batch.allocations().device_allocations,1u);
  for(std::size_t e=0;e<initial.size();++e) {
    SCOPED_TRACE(e);const auto& result=initial[e];EXPECT_TRUE(result.history.active);
    EXPECT_EQ(result.frame.length_m,rig.input.model.references()[e].length_m);EXPECT_GT(result.critical_dt_s,2*rig.initial.h);
    EXPECT_TRUE(tl::math::fixed3::Orthonormal(result.frame.axes));EXPECT_TRUE(tl::math::fixed3::Orthonormal(result.frame.midpoint_axes));
    for(const auto& rhs:result.endpoints){EXPECT_EQ(tl::math::fixed3::Norm(rhs.force_N),0);EXPECT_EQ(tl::math::fixed3::Norm(rhs.couple_Nm),0);}
  }
  spring::batch_detail::ArenaLayout layout;ASSERT_TRUE(spring::batch_detail::MakeLayout(1,1024,5,2*1024*1024,layout));
  EXPECT_EQ(rig.batch.allocations().device_bytes,layout.bytes);
}
TEST_F(Type25BatchCuda, ExactResidentAndHostCapsRejectBeforeAllocationAndPermitCleanRetry) {
  Rig rig;ASSERT_TRUE(rig.Initialize());const auto config=rig.input.Config(rig.owner.accepted());
  for(unsigned mode=0;mode<5;++mode) {
    spring::Batch candidate;auto bad=config;
    if(mode==0)bad.max_device_bytes=rig.batch.allocations().device_bytes-1;
    if(mode==1)bad.max_host_bytes=rig.batch.host_bytes()-1;
    if(mode==2)bad.max_connections=128;
    if(mode==3)bad.element_count=SIZE_MAX;
    if(mode==4)bad.owner.node_count=SIZE_MAX;
    EXPECT_EQ(candidate.InitializeJoined(bad,rig.input.model,rig.input.mass).status,spring::BatchStatus::ResourceLimit);
    EXPECT_EQ(candidate.allocations().device_allocations,0u);EXPECT_EQ(candidate.allocations().device_bytes,0u);EXPECT_EQ(candidate.host_bytes(),0u);
    auto exact=config;exact.max_device_bytes=rig.batch.allocations().device_bytes;exact.max_host_bytes=rig.batch.host_bytes();
    const auto retry=candidate.InitializeJoined(exact,rig.input.model,rig.input.mass);EXPECT_EQ(retry.status,spring::BatchStatus::Success)<<retry.message;
  }
  spring::Batch wrong;auto wrong_config=config;++wrong_config.element_count;
  EXPECT_EQ(wrong.InitializeJoined(wrong_config,rig.input.model,rig.input.mass).status,spring::BatchStatus::InvalidInput);
  EXPECT_EQ(wrong.allocations().device_bytes,0u);
}
} // namespace type25_batch_test
