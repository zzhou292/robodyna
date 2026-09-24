// SPDX-License-Identifier: AGPL-3.0-or-later
#include "GpuFixture.h"
#include "lib_src/collision/represented_interval_crossing/NativeStorageQualification.h"
#include <cmath>
#include <limits>

namespace native_gpu_test {
namespace {
namespace native = c::represented_interval_crossing;
using C = c::RepresentedIntervalClassification;
using R = c::RepresentedIntervalReason;

void CompareWide(c::RepresentedIntervalCrossing& cpu,
    c::RepresentedIntervalCrossingGpu& gpu, const Cases& cases,
    const c::RepresentedIntervalGpuLimits& limits, cudaStream_t stream,
    bool device_expected) {
  ASSERT_EQ(cases.paths.size(),2u);
  ASSERT_EQ(cases.pairs.size(),1u);
  const auto independent=native::CompareNativeStorage(
      cases.paths[0],cases.paths[1],limits.native);
  ASSERT_EQ(independent.status,S::Ok);
  EXPECT_EQ(independent.original.counters.wide_pairs,1u);
  fixture::SameResult(independent.original.result,independent.current.result);
  const auto result=Compare(cpu,gpu,cases,stream);
  ASSERT_EQ(result.native.status,S::Ok);
  ASSERT_EQ(result.device.status,D::Ok);
  ASSERT_TRUE(gpu.results().complete);
  ASSERT_EQ(gpu.results().count,1u);
  fixture::SameResult(gpu.results().data[0],independent.original.result);
  EXPECT_EQ(result.device.device_pairs,device_expected?1u:0u);
  EXPECT_EQ(result.device.consumed_device_pairs,device_expected?1u:0u);
  EXPECT_EQ(result.device.host_pairs,device_expected?0u:1u);
  EXPECT_EQ(result.device.batches,device_expected?1u:0u);
  EXPECT_EQ(result.device.scene_uploads,device_expected?1u:0u);
}
}

TEST(NativeGpuZeroStorageCuda, OriginalZeroFamiliesMatchIndependentWideResultsAtEveryDepthAndBudget) {
  Streams streams;
  const std::array<c::Vec3,3> point{};
  for (unsigned depth:{0u,20u,52u}) for(std::size_t work:{1u,31u}) {
    SCOPED_TRACE(depth);SCOPED_TRACE(work);
    auto limits=native_gpu_test::Limits();
    limits.native.max_depth=depth;
    limits.native.max_work_per_pair=work;
    auto cpu=fixture::Owner(limits.native);
    c::RepresentedIntervalCrossingGpu gpu;
    ASSERT_EQ(gpu.Initialize(limits,streams.first).native.status,S::Ok);
    for(unsigned family=0;family<6;++family) {
      SCOPED_TRACE(family);
      Cases cases;
      const auto a=fixture::BaseTriangle();
      if(family==0)Add(cases,a,a,a);
      if(family==1)Add(cases,a,fixture::BaseTriangle(1),fixture::BaseTriangle(1));
      if(family==2)Add(cases,a,fixture::BaseTriangle(1),fixture::BaseTriangle(-3));
      if(family==3)Add(cases,a,fixture::BaseTriangle(1),fixture::BaseTriangle(1-4096));
      if(family==4) {
        const std::array<c::Vec3,3> b{{{2,0,0},{3,0,0},{2,-1,0}}};
        Add(cases,a,b,b);
      }
      if(family==5)Add(cases,a,point,point);
      const auto domain=native::NativeStorageDomain::FromPaths(
          cases.paths[0],cases.paths[1],depth).report();
      ASSERT_GT(domain.projection.coordinate_bits,125u);
      ASSERT_TRUE(domain.eligible);
      CompareWide(cpu,gpu,cases,limits,streams.first,true);
    }
  }
}

TEST(NativeGpuZeroStorageCuda, SignedZerosAndAllZeroDegeneracyRemainExactOnDevice) {
  Streams streams;auto limits=native_gpu_test::Limits();
  auto cpu=fixture::Owner(limits.native);c::RepresentedIntervalCrossingGpu gpu;
  ASSERT_EQ(gpu.Initialize(limits,streams.first).native.status,S::Ok);
  c::RepresentedIntervalResult positive;
  for(unsigned sign=0;sign<2;++sign) {
    auto triangle=fixture::BaseTriangle();
    for(auto& p:triangle) {
      if(p.x==0)p.x=sign?-0.:0.;
      if(p.y==0)p.y=sign?-0.:0.;
      if(p.z==0)p.z=sign?-0.:0.;
    }
    Cases cases;Add(cases,triangle,triangle,triangle);
    CompareWide(cpu,gpu,cases,limits,streams.first,true);
    ASSERT_EQ(gpu.results().count,1u);
    if(!sign)positive=gpu.results().data[0];
    else fixture::SameResult(positive,gpu.results().data[0]);
  }
  const std::array<c::Vec3,3> zero{};
  Cases collapsed;Add(collapsed,zero,zero,zero);
  CompareWide(cpu,gpu,collapsed,limits,streams.first,true);
  ASSERT_EQ(gpu.results().count,1u);
  EXPECT_EQ(gpu.results().data[0].classification,C::Unresolved);
  EXPECT_EQ(gpu.results().data[0].reason,R::DegenerateGeometry);
}

TEST(NativeGpuZeroStorageCuda, NonzeroBitBoundaryRemains125Versus126WhenOtherCoordinatesAreZero) {
  Streams streams;
  for(unsigned depth:{0u,20u,52u}) {
    auto limits=native_gpu_test::Limits();limits.native.max_depth=depth;
    auto cpu=fixture::Owner(limits.native);c::RepresentedIntervalCrossingGpu gpu;
    ASSERT_EQ(gpu.Initialize(limits,streams.first).native.status,S::Ok);
    for(unsigned bits:{125u,126u}) {
      SCOPED_TRACE(depth);SCOPED_TRACE(bits);
      auto triangle=Positive();
      triangle[0].z=std::ldexp(1.,55+int(depth+1)-int(bits));
      triangle[1].z=0;
      Cases cases;Add(cases,triangle,triangle,triangle);
      const auto domain=native::NativeStorageDomain::FromPaths(
          cases.paths[0],cases.paths[1],depth).report();
      EXPECT_GT(domain.projection.coordinate_bits,125u);
      ASSERT_EQ(domain.storage.coordinate_bits,bits);
      ASSERT_EQ(domain.eligible,bits==125);
      CompareWide(cpu,gpu,cases,limits,streams.first,bits==125);
    }
  }
}

TEST(NativeGpuZeroStorageCuda, SubnormalAndHugeZeroCoordinateScalesKeepWideClassificationAndShortcutDomain) {
  Streams streams;auto limits=native_gpu_test::Limits();
  auto cpu=fixture::Owner(limits.native);c::RepresentedIntervalCrossingGpu gpu;
  ASSERT_EQ(gpu.Initialize(limits,streams.first).native.status,S::Ok);
  for(int exponent:{-1074,-1070,-1000,0,700,1020}) {
    SCOPED_TRACE(exponent);
    auto a=fixture::BaseTriangle(),b=fixture::BaseTriangle(1);
    for(auto* triangle:{&a,&b})for(auto& point:*triangle) {
      point.x=std::ldexp(point.x,exponent);
      point.y=std::ldexp(point.y,exponent);
      point.z=std::ldexp(point.z,exponent);
    }
    Cases cases;Add(cases,a,b,b);
    const auto domain=native::NativeStorageDomain::FromPaths(
        cases.paths[0],cases.paths[1],limits.native.max_depth).report();
    ASSERT_TRUE(domain.eligible);
    if(exponent>=700)EXPECT_FALSE(domain.projection.eligible);
    CompareWide(cpu,gpu,cases,limits,streams.first,true);
    ASSERT_EQ(gpu.results().count,1u);
    EXPECT_EQ(gpu.results().data[0].classification,C::CertifiedSeparated);
  }
}

TEST(NativeGpuZeroStorageCuda, GenuineNonzeroWideMixtureStillUsesHostAndValidDeviceRetryRemainsAvailable) {
  Streams streams;auto limits=native_gpu_test::Limits();
  auto cpu=fixture::Owner(limits.native);c::RepresentedIntervalCrossingGpu gpu;
  ASSERT_EQ(gpu.Initialize(limits,streams.first).native.status,S::Ok);
  Cases wide;Add(wide,fixture::BaseTriangle(),fixture::BaseTriangle(1),fixture::BaseTriangle(1));
  RequireNonzeroWideStorage(wide.paths);
  for(const auto& path:wide.paths)for(const auto& vertex:path.vertices)for(const auto point:vertex.endpoint)
    ASSERT_TRUE(point.x!=0 && point.y!=0 && point.z!=0);
  const auto domain=native::NativeStorageDomain::FromPaths(
      wide.paths[0],wide.paths[1],limits.native.max_depth).report();
  ASSERT_GT(domain.storage.coordinate_bits,125u);
  ASSERT_FALSE(domain.eligible);
  CompareWide(cpu,gpu,wide,limits,streams.first,false);
  Cases zero;Add(zero,fixture::BaseTriangle(),fixture::BaseTriangle(),fixture::BaseTriangle());
  CompareWide(cpu,gpu,zero,limits,streams.first,true);
  const auto before=gpu.results();const auto saved=Copy(before);
  auto invalid=zero;
  invalid.paths[1].vertices[2].endpoint[1].x=std::numeric_limits<double>::quiet_NaN();
  const auto rejected=Compare(cpu,gpu,invalid,streams.first);
  EXPECT_EQ(rejected.native.status,S::InvalidInput);
  EXPECT_EQ(rejected.device.status,D::NotInvoked);
  Preserved(before,saved,gpu.results());
  CompareWide(cpu,gpu,zero,limits,streams.first,true);
}
}  // namespace native_gpu_test
