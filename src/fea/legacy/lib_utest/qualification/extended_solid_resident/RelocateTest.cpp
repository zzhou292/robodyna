// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/materials/law90/Relocate.h"
namespace extended_resident_test {
TEST(ExtendedResidentRelocation, PreservesAllPreparedScalarsAndRejectsBadRangesAtomically) {
  namespace law = tl::material::law90;
  Fixture fixture;
  const auto source = fixture.input90[0].material;
  double x[3]{0,.2,.4}, y[3]{0,10e6,50e6};
  law::PreparedMaterial relocated;
  ASSERT_EQ(law::RelocatePreparedCurve(source,{x,y,3},relocated),law::Status::Ok);
  law::PreparedMaterial expected;
  ASSERT_EQ(law::PrepareSI(fixture.foam_input,{x,y,3},expected),law::Status::Ok);
  EXPECT_TRUE(law::SamePreparedMaterial(relocated,expected));
  for (const auto bad : {law::CurveView{x,y,2},law::CurveView{nullptr,y,3},
      law::CurveView{x,x,3},law::CurveView{x,x+1,3},
      law::CurveView{reinterpret_cast<double*>(UINTPTR_MAX-7),y,3},
      law::CurveView{reinterpret_cast<double*>(reinterpret_cast<unsigned char*>(x)+1),y,3}}) {
    EXPECT_EQ(law::RelocatePreparedCurve(source,bad,relocated),law::Status::InvalidInput);
    EXPECT_TRUE(law::SamePreparedMaterial(relocated,expected));
  }
  auto inplace = source;
  EXPECT_EQ(law::RelocatePreparedCurve(inplace,{x,y,3},inplace),law::Status::InvalidInput);
  EXPECT_TRUE(law::SamePreparedMaterial(inplace,source));
  const auto* alias = reinterpret_cast<const double*>(&relocated);
  EXPECT_EQ(law::RelocatePreparedCurve(source,{alias,y,3},relocated),law::Status::InvalidInput);
  EXPECT_TRUE(law::SamePreparedMaterial(relocated,expected));
}
TEST(ExtendedResidentRelocation, ThreeOwnedPoolsRebaseWithoutBorrowedLifetimeOrDeviceReads) {
  Fixture fixture;
  const auto domain = fixture.Domain();
  s::Model model;
  ASSERT_TRUE(model.Initialize(domain,fixture.Input()));
  const auto config = Config(domain.node_count());
  d::ArenaLayout layout;
  ASSERT_TRUE(d::Plan(config,model,layout));
  fixture.stress[2] = fixture.rear_y[2] = fixture.foam_y[2] = std::numeric_limits<double>::quiet_NaN();
  tl::util::HostArena upload;
  ASSERT_TRUE(upload.Initialize(layout.bytes));
  d::Storage header;
  ASSERT_TRUE(d::BuildUpload(config,model,upload,layout,header));
  EXPECT_EQ(header.material36[0].curve.yield_stress_pa[2],3e6);
  EXPECT_EQ(header.material44[0].curve.yield_stress_pa[2],450e6);
  EXPECT_EQ(header.material90[0].curve().stress_pa[2],50e6);
  // Opaque, aligned device-address analogue: rebasing must never read it.
  auto* device = reinterpret_cast<void*>(std::uintptr_t(0x100000000));
  ASSERT_TRUE(d::RebaseCurves(model,layout,device,header));
  auto* curves = tl::util::ArenaPointer<double>(device,layout.curves);
  EXPECT_EQ(header.material36[0].curve.plastic_strain,curves);
  EXPECT_EQ(header.material44[0].curve.plastic_strain,curves+6);
  EXPECT_EQ(header.material90[0].curve().compression_strain,curves+12);
  EXPECT_EQ(header.material90[0].curve().stress_pa,curves+15);
  EXPECT_EQ(model.materials90()[0].value.curve().stress_pa[2],50e6);
  EXPECT_FALSE(header.solid18_law44.slab[0][0].history.prepared());
  EXPECT_FALSE(header.solid18_law90.slab[0][0].history.legacy()->prepared());
}
TEST(ExtendedResidentRelocation, ActualBlankHuAndExplicitHuOneKeepTheirPreparedBranch) {
  namespace law=tl::material::law90;
  for(int flag:{1,2}) {
    Fixture fixture;fixture.foam_input.hysteresis=flag==1?0:1;fixture.PrepareFoam();
    const auto source=fixture.input90[0].material;
    double x[3]{0,.2,.4},y[3]{0,10e6,50e6};
    law::PreparedMaterial relocated,expected;
    ASSERT_EQ(law::RelocatePreparedCurve(source,{x,y,3},relocated),law::Status::Ok);
    ASSERT_EQ(law::PrepareSI(fixture.foam_input,{x,y,3},expected),law::Status::Ok);
    EXPECT_EQ(relocated.reader().loading_flag,flag);
    EXPECT_TRUE(law::SamePreparedMaterial(relocated,expected));
  }
}
} // namespace extended_resident_test
