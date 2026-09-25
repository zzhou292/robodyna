// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "CudaFixture.h"
#include "../radioss_type25_fixed_main_startup/GeneralCases.h"
#include <limits>
namespace type25_current_normals_test {
namespace {
using GeneralFixture=FixtureT<type25_startup_test::GeneralBuilt>;
using GeneralNormalsCuda=device::CurrentNormalsCuda;
void SameGeneralStages(const device::Observation& actual,const NativeResult& native,const Result& host) {
  ASSERT_EQ(actual.report.status,c::Status::Ok);ASSERT_TRUE(native.finite);ASSERT_TRUE(actual.values.finite);
  Same(host,native);SameNormals(actual.values.flag1_normals,native.flag1_normals);
  SameNormals(actual.values.normals,native.normals);SameReferences(actual.values.references,native.references);
  EXPECT_EQ(actual.values.primary_skip,native.primary_skip);
}
}
TEST_F(GeneralNormalsCuda, GenuineGeneralStartupFeedsNativeHostAndCudaRecurrences) {
  using namespace type25_startup_test;
  const std::vector<Case> sources{EdgeStar(3),EdgeStar(4,2,true),DisconnectedFan(),SharedExtraVertex(false),SharedExtraVertex(true)};
  for(std::size_t shape=0;shape<sources.size();++shape)for(unsigned threads:{1u,7u,32u,64u})for(bool reverse:{false,true}) {
    SCOPED_TRACE(shape);
    SCOPED_TRACE(threads);
    SCOPED_TRACE(reverse);
    GeneralFixture f(sources[shape]);
    ASSERT_EQ(f.built.startup.topology,s::TopologyPolicy::NativeOrdinaryShell);
    ASSERT_EQ(f.built.startup.profile,s::Profile::OrdinaryExteriorMovingMain);
    // Both topologies and their first caches are built from the original mesh:
    // C++ BuildStarter for the trial path, full original NEIGH/NORM for the oracle.
    // No neighbor editing or fabricated normal/reference data occurs here.
    for(unsigned step=0;step<4;++step) {
      SCOPED_TRACE(step);
      for(std::size_t i=0;i<f.mesh.ids.size();++i) {
        const double x=f.mesh.positions[3*i],y=f.mesh.positions[3*i+1],z=f.mesh.positions[3*i+2];
        f.positions[3*i]=x+.025*step*y;f.positions[3*i+1]=y+.125*step*z;
        f.positions[3*i+2]=z+.03125*step*x*y;
      }
      if(step==0)f.AllActive();
      else if(step==1)f.GeneratedMasks({1});
      else if(step==2)f.GeneratedMasks({},1,{2});
      else {
        f.coefficients[0]=0;f.coefficients[f.mesh.primary.size()]=0;
        f.RefreshFree();f.GeneratedMasks();
      }
      const auto expected=Oracle(f.Input(true));const auto host=EvaluateHostNormals(f.Input());
      const auto actual=EvaluateDevice(f.Input(),threads,reverse,GeneralFixture::Limits());
      SameGeneralStages(actual,expected,host);
      f.prior=actual.values.normals;f.native_prior=expected.normals;
    }
  }
}
TEST_F(GeneralNormalsCuda, SplitReferenceCapacityAndInvalidCurrentDataPreserveThenRetry) {
  GeneralFixture f(type25_startup_test::DisconnectedFan());
  const auto expected=Oracle(f.Input(true));const auto host=EvaluateHostNormals(f.Input());
  SameGeneralStages(EvaluateDevice(f.Input(),32,false,GeneralFixture::Limits()),expected,host);
  auto cap=GeneralFixture::Limits();cap.references=f.built.startup.starter.reference_count-1;
  const auto limited=EvaluateDevice(f.Input(),7,true,cap);
  EXPECT_EQ(limited.report.status,c::Status::ResourceLimit);EXPECT_TRUE(limited.publication_unchanged);
  const auto good=f.positions[0];f.positions[0]=std::numeric_limits<double>::quiet_NaN();
  const auto invalid=EvaluateDevice(f.Input(),64,true,GeneralFixture::Limits());
  EXPECT_EQ(invalid.report.status,c::Status::InvalidInput);EXPECT_TRUE(invalid.publication_unchanged);
  f.positions[0]=good;
  SameGeneralStages(EvaluateDevice(f.Input(),1,true,GeneralFixture::Limits()),expected,host);
}
} // namespace type25_current_normals_test
