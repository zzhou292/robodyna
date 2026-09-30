#include "NativeOracle.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cstring>
namespace qbat_test {
template<std::size_t N>
void Agreement(const std::array<double,N>& actual,const std::array<double,N>& native) {
  for(std::size_t i=0;i<N;++i) {
    ASSERT_TRUE(std::isfinite(native[i])) << i;
    const double scale=std::max(std::abs(native[i]),std::abs(actual[i]));
    EXPECT_NEAR(actual[i],native[i],3e-12*scale+1e-24) << i;
  }
}
TEST(QbatNative, StartupFrameMassAndCndleniAcrossResolvedViscosities) {
  for(bool warped:{false,true}) for(double viscosity:{0.,.001,.035}) {
    auto input=Fixture(warped);
    input.options.membrane_viscosity=viscosity;
    input.options.numerical_viscosity=viscosity*.25;
    qb::Reference reference;
    ASSERT_EQ(qb::InitializeReference(input,reference),qb::Status::kSuccess);
    Agreement(Values(reference),NativeReference(input));
    // The 5/4 QEPH length branch is measurably different, even on a rectangle.
    auto current=Current(input);
    tl::fea::qeph::PrescribedInterval q_interval;
    for(unsigned i=0;i<4;++i) q_interval.position_endpoint[i]=current.position_m[i];
    tl::fea::qeph::detail::GeometryWork wrong;
    ASSERT_EQ(tl::fea::qeph::detail::CurrentGeometry(q_interval,wrong),qb::Status::kSuccess);
    EXPECT_GT(std::abs(wrong.values.characteristic_length/reference.coefficients().characteristic_length_m-1),.01);
  }
}
TEST(QbatNative, FourGaussCellsThroughLargeRigidMotionAndDeformation) {
  const auto input=Fixture();
  qb::Reference reference;
  ASSERT_EQ(qb::InitializeReference(input,reference),qb::Status::kSuccess);
  for(unsigned step=0;step<96;++step) {
    SCOPED_TRACE(step);
    auto current=Current(input);
    for(auto& x:current.position_m) {
      x.x*=1+.0007*step;
      x.y+=.04*std::sin(.07*step)*x.x;
      x=Transform(x,.019*step,.0002*step);
    }
    qb::Geometry actual;
    ASSERT_EQ(qb::EvaluateGeometry(reference,current,actual),qb::Status::kSuccess);
    int flat=0;
    Agreement(Values(actual),NativeGeometry(current,flat));
    EXPECT_EQ(flat,1); // Native NPTT1 selects flat algorithm despite actual warp.
    EXPECT_GT(std::abs(actual.actual_warpage_m),1e-5);
  }
}
TEST(QbatNative, LargeWorldTranslationRetainsNativeCurrentFrameExpressionOrder) {
  auto input=Fixture();
  for(auto& x:input.quadrilateral.position) x=Transform(x,.71,512);
  qb::Reference reference;
  ASSERT_EQ(qb::InitializeReference(input,reference),qb::Status::kSuccess);
  Agreement(Values(reference),NativeReference(input));
  qb::Geometry actual;
  auto current=Current(input);
  ASSERT_EQ(qb::EvaluateGeometry(reference,current,actual),qb::Status::kSuccess);
  int flat=0;
  Agreement(Values(actual),NativeGeometry(current,flat));
  // Different native starter versus engine normalizations remain separate.
  bool differs=false;
  for(unsigned i=0;i<9;++i) differs|=actual.frame.v[i]!=reference.quadrilateral().frame.v[i];
  EXPECT_TRUE(differs);
}
TEST(QbatNative, RejectedCandidateKeepsPreviousGeometryAndRetriesNativePacket) {
  const auto input=Fixture();
  qb::Reference reference;
  ASSERT_EQ(qb::InitializeReference(input,reference),qb::Status::kSuccess);
  auto current=Current(input);
  qb::Geometry output;
  ASSERT_EQ(qb::EvaluateGeometry(reference,current,output),qb::Status::kSuccess);
  const auto before=output;
  current.position_m[2]=current.position_m[0];
  ASSERT_NE(qb::EvaluateGeometry(reference,current,output),qb::Status::kSuccess);
  EXPECT_EQ(std::memcmp(&output,&before,sizeof(output)),0);
  current=Current(input);
  current.position_m[2].z+=.0001;
  ASSERT_EQ(qb::EvaluateGeometry(reference,current,output),qb::Status::kSuccess);
  int flat=0;
  Agreement(Values(output),NativeGeometry(current,flat));
}
} // namespace qbat_test
