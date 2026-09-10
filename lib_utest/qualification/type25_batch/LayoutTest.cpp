// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Input.h"
#include "lib_src/elements/type25/Type25BatchStorage.h"

namespace type25_batch_test {
TEST(Type25BatchLayout, ExactActiveCountsAndOneByteFailurePreserveLayout) {
  namespace b=spring::batch_detail;
  for(std::size_t count:{1u,64u,129u,1024u}) {
    b::ArenaLayout layout;ASSERT_TRUE(b::MakeLayout(1,count,2048,2*1024*1024,layout));
    EXPECT_EQ(layout.properties.count,1u);EXPECT_EQ(layout.elements.count,count);EXPECT_EQ(layout.nodes.count,2048u);
    EXPECT_EQ(layout.slab[0].count,count);EXPECT_EQ(layout.slab[1].count,count);EXPECT_EQ(layout.status.count,count);
    b::ArenaLayout output;output.bytes=271;
    EXPECT_FALSE(b::MakeLayout(1,count,2048,layout.bytes-1,output));EXPECT_EQ(output.bytes,271u);
    EXPECT_TRUE(b::MakeLayout(1,count,2048,layout.bytes,output));EXPECT_EQ(output.bytes,layout.bytes);
    tl::util::HostArena host;ASSERT_TRUE(host.Initialize(layout.bytes));
    const auto header=b::RebasedHeader(host.data(),layout);
    EXPECT_EQ(header.model.elements,tl::util::ArenaPointer<b::DeviceElement>(host.data(),layout.elements));
    EXPECT_EQ(header.slab[1].element,tl::util::ArenaPointer<spring::Evaluation>(host.data(),layout.slab[1]));
  }
  b::ArenaLayout sentinel;sentinel.bytes=97;
  for(auto count:{std::size_t{0},std::size_t{1025},SIZE_MAX}) {
    EXPECT_FALSE(b::MakeLayout(1,count,5,2*1024*1024,sentinel));EXPECT_EQ(sentinel.bytes,97u);
  }
}
TEST(Type25BatchLayout, OriginalGeometryInitialCacheHasNativeBoundsWithoutDtZeroForceCall) {
  Input input;ASSERT_TRUE(input.Initialize(129));fe::NodalStamp stamp;stamp.owner_id=1;stamp.node_count=5;
  stamp.fixed_dt=0x1p-20;stamp.has_rotations=true;stamp.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  const auto config=input.Config(stamp);spring::batch_detail::ArenaLayout layout;
  ASSERT_TRUE(spring::batch_detail::MakeLayout(1,129,5,config.max_device_bytes,layout));
  tl::util::HostArena arena;ASSERT_TRUE(arena.Initialize(layout.bytes));spring::batch_detail::Storage header;
  spring::BatchDiagnostics diagnostics;
  ASSERT_EQ(spring::batch_detail::BuildStartup(config,input.model,input.mass,arena,layout,header,diagnostics).status,spring::BatchStatus::Success);
  EXPECT_TRUE(diagnostics.valid);EXPECT_EQ(diagnostics.active_count,129u);EXPECT_EQ(diagnostics.epoch,0u);
  EXPECT_FALSE(diagnostics.has_completed_interval);EXPECT_FALSE(diagnostics.accepted_force_assembled);
  double minimum=1e30;
  for(std::size_t e=0;e<129;++e) {
    const auto& result=header.slab[0].element[e];const auto& reference=input.model.references()[e];
    EXPECT_TRUE(result.history.active);EXPECT_EQ(result.frame.length_m,reference.length_m);
    EXPECT_EQ(result.frame.midpoint_length_m,reference.length_m);EXPECT_TRUE(tl::math::fixed3::Orthonormal(result.frame.axes));
    EXPECT_TRUE(tl::math::fixed3::Orthonormal(result.frame.midpoint_axes));
    spring::Stability expected;ASSERT_EQ(spring::CriticalStep(input.model.source_units(),input.property.property,reference.length_m,expected),spring::Status::Success);
    EXPECT_DOUBLE_EQ(result.critical_dt_s,expected.critical_dt_s);minimum=std::min(minimum,expected.critical_dt_s);
    for(const auto& rhs:result.endpoints) {
      EXPECT_EQ(tl::math::fixed3::Norm(rhs.force_N),0);EXPECT_EQ(tl::math::fixed3::Norm(rhs.couple_Nm),0);
    }
    for(double work:result.history.internal_work_J)EXPECT_EQ(work,0);
  }
  EXPECT_DOUBLE_EQ(diagnostics.minimum_native_dt,minimum);
  // Startup must not accept an endpoint-phase description as original TT=0.
  auto wrong=config;wrong.owner.epoch=1;spring::batch_detail::Storage untouched;untouched.model.source_instance_id=991;
  spring::BatchDiagnostics output;output.attempt=37;
  EXPECT_EQ(spring::batch_detail::BuildStartup(wrong,input.model,input.mass,arena,layout,untouched,output).status,spring::BatchStatus::InvalidInput);
  EXPECT_EQ(untouched.model.source_instance_id,991u);EXPECT_EQ(output.attempt,37u);
}
} // namespace type25_batch_test
