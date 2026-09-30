// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"
#include "NativeChecks.h"
#include "../solid_model/OriginalFixture.h"

namespace solid_resident_test {
TEST_F(SolidResidentCuda, All2412SourceConstructorValuesFitAndMatchNativeWithoutOwnerAdmission) {
  solid_model_test::OriginalFixture fixture;
  auto config=Config(fixture.domain.node_count());
  s::BatchForecast forecast;
  ASSERT_TRUE(s::Batch::Forecast(config,fixture.model,forecast));
  config.limits.max_device_bytes=forecast.device_bytes;
  config.limits.max_host_bytes=forecast.startup_host_bytes;
  s::Batch batch;
  ASSERT_TRUE(Good(batch.InitializeJoined(config,fixture.model)));
  std::vector<s::Result18> a(fixture.a.size());
  std::vector<s::Result24> b(fixture.b.size());
  std::vector<s::Result6z> c(fixture.c.size());
  s::BatchDiagnostics diagnostics;
  const s::ResultBuffers output{a.data(),a.size(),b.data(),b.size(),c.data(),c.size()};
  EXPECT_EQ(batch.CopyAcceptedResults(config.owner,output,&diagnostics).status,s::BatchStatus::NotBound);
  ASSERT_TRUE(Good(Peer::ReadConstructed(batch,output,diagnostics)));
  EXPECT_EQ(diagnostics.epoch,0u);
  EXPECT_FALSE(diagnostics.has_completed_interval);
  EXPECT_FALSE(diagnostics.accepted_force_assembled);
  for (std::size_t i=0;i<a.size();++i) {
    const auto& input=fixture.a[i];
    SCOPED_TRACE(input.reference.input().source_element_id);
    const auto initial=solid18_force_test::NativeInitial(input.reference.input());
    fe::solid18::PrescribedInterval interval;
    for (unsigned n=0;n<8;++n) interval.position_endpoint_m[n]=input.reference.input().position_m[n];
    CheckNative(a[i],solid18_force_test::Native(input.material,initial,interval));
    ASSERT_FALSE(HasFailure());
    EXPECT_EQ(a[i].stamp.sample_index,0u);
  }
  for (std::size_t i=0;i<b.size();++i) {
    const auto& input=fixture.b[i];
    SCOPED_TRACE(input.reference.input().source_element_id);
    const auto initial=heph_test::InitializeNative(input.reference.input());
    fe::solid24::PrescribedInterval interval;
    for (unsigned n=0;n<8;++n) interval.position_m[n]=input.reference.input().position_m[n];
    CheckNative(b[i],heph_test::NativeStep(initial,interval,input.material));
    ASSERT_FALSE(HasFailure());
    EXPECT_EQ(b[i].stamp.sample_index,0u);
  }
  for (std::size_t i=0;i<c.size();++i) {
    const auto& input=fixture.c[i];
    SCOPED_TRACE(input.reference.input().source_element_id);
    solid6z_force_test::NativeHistory initial;
    ASSERT_TRUE(initial.Initialize(input.reference.input(),input.material));
    CheckNative(c[i],initial.InitializeForce({}),input.reference.geometry().volume_m3);
    ASSERT_FALSE(HasFailure());
    EXPECT_EQ(c[i].stamp.sample_index,0u);
  }
  EXPECT_EQ(batch.allocations().device_bytes,forecast.device_bytes);
  RecordProperty("device_bytes",forecast.device_bytes);
  RecordProperty("startup_host_bytes",forecast.startup_host_bytes);
  RecordProperty("source_parents",a.size()+b.size()+c.size());
}
} // namespace solid_resident_test
