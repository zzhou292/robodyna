#include "NativeOracle.h"
#include "source_fixture/YarisQbatSourceFixture.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cstring>
#include <iterator>

namespace qbat_test {
namespace original=yaris_qbat_source_fixture;
qb::ReferenceInput OriginalInput(const original::Quad& quad) {
  qb::ReferenceInput input;
  auto& q=input.quadrilateral;
  for(unsigned i=0;i<4;++i) {
    const auto& node=original::nodes[quad.node[i]];
    q.node_ids[i]=node.id;
    q.position[i]={node.position_m[0],node.position_m[1],node.position_m[2]};
  }
  // Authenticated original MAT024/SECTION_SHELL declaration: only its virgin
  // geometric mass/stiffness inputs are used here. No material/failure admission.
  q.density=1000;
  q.young_modulus=250e6;
  q.poisson_ratio=.35;
  q.thickness=original::thickness_m;
  input.initial_a11_pa=q.young_modulus/(1-q.poisson_ratio*q.poisson_ratio);
  return input;
}
template<std::size_t N>
void Agreement(const std::array<double,N>& actual,const std::array<double,N>& native) {
  for(std::size_t i=0;i<N;++i) {
    ASSERT_TRUE(std::isfinite(native[i])) << i;
    EXPECT_NEAR(actual[i],native[i],3e-12*std::max(std::abs(actual[i]),std::abs(native[i]))+1e-24) << i;
  }
}
TEST(QbatOriginal, EveryOriginalMidlayerQuadHasIndependentNativeReferenceAndFourPoints) {
  ASSERT_EQ(std::size(original::nodes),4384u);
  ASSERT_EQ(std::size(original::quads),4250u);
  ASSERT_EQ(original::part_id,2000524u);
  ASSERT_DOUBLE_EQ(original::thickness_m,.0005);
  double mass=0,min_dt=1,maximum_warp=0;
  for(const auto& quad:original::quads) {
    SCOPED_TRACE(quad.id);
    const auto input=OriginalInput(quad);
    qb::Reference reference;
    ASSERT_EQ(qb::InitializeReference(input,reference),qb::Status::kSuccess);
    Agreement(Values(reference),NativeReference(input));
    qb::Geometry geometry;
    ASSERT_EQ(qb::EvaluateGeometry(reference,Current(input),geometry),qb::Status::kSuccess);
    int flat=0;
    Agreement(Values(geometry),NativeGeometry(Current(input),flat));
    EXPECT_EQ(flat,1);
    mass+=4*reference.quadrilateral().nodal_mass[0];
    min_dt=std::min(min_dt,reference.coefficients().unscaled_element_dt_s);
    maximum_warp=std::max(maximum_warp,std::abs(geometry.actual_warpage_m));
  }
  EXPECT_GT(mass,0);
  EXPECT_GT(min_dt,0);
  EXPECT_GT(maximum_warp,1e-7);
  RecordProperty("quad_mass_kg",std::to_string(mass));
  RecordProperty("minimum_native_dt_s",std::to_string(min_dt));
}
TEST(QbatOriginal, FirstLastAndMostWarpedParentsRetainLargeCurrentMotionAndRetry) {
  const original::Quad* most_warped=nullptr;
  double maximum=-1;
  for(const auto& quad:original::quads) {
    const auto input=OriginalInput(quad);
    qb::Reference reference;
    ASSERT_EQ(qb::InitializeReference(input,reference),qb::Status::kSuccess);
    qb::Geometry geometry;
    ASSERT_EQ(qb::EvaluateGeometry(reference,Current(input),geometry),qb::Status::kSuccess);
    if(std::abs(geometry.actual_warpage_m)>maximum) {
      maximum=std::abs(geometry.actual_warpage_m);
      most_warped=&quad;
    }
  }
  ASSERT_NE(most_warped,nullptr);
  for(const auto* quad:{&original::quads[0],most_warped,&original::quads[std::size(original::quads)-1]}) {
    SCOPED_TRACE(quad->id);
    const auto input=OriginalInput(*quad);
    qb::Reference reference;
    ASSERT_EQ(qb::InitializeReference(input,reference),qb::Status::kSuccess);
    for(unsigned step=0;step<32;++step) {
      auto current=Current(input);
      for(auto& x:current.position_m) x=Transform(x,.04*step,.003*step);
      qb::Geometry output;
      ASSERT_EQ(qb::EvaluateGeometry(reference,current,output),qb::Status::kSuccess);
      int flat=0;
      Agreement(Values(output),NativeGeometry(current,flat));
      const auto accepted=Values(output);
      auto bad=current;
      bad.position_m[3]=bad.position_m[1];
      EXPECT_NE(qb::EvaluateGeometry(reference,bad,output),qb::Status::kSuccess);
      EXPECT_EQ(Values(output),accepted);
      ASSERT_EQ(qb::EvaluateGeometry(reference,current,output),qb::Status::kSuccess);
      EXPECT_EQ(Values(output),accepted);
    }
  }
}
} // namespace qbat_test
