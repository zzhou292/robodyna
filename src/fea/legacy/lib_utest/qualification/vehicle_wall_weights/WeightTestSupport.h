#pragma once
#include "lib_src/collision/NodalWallWeightStartup.h"
#include "lib_src/collision/SurfaceMaterialMeasure.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cstring>
#include <vector>

namespace vehicle_wall_weight_test {
namespace sc=tlfea::contact;
inline void Same(double a,double b) {
  std::uint64_t x=0,y=0;std::memcpy(&x,&a,8);std::memcpy(&y,&b,8);EXPECT_EQ(x,y);
}
inline void Same(sc::Q4CertifiedIntegral a,sc::Q4CertifiedIntegral b) {
  Same(a.value,b.value);Same(a.error,b.error);Same(a.lower,b.lower);Same(a.upper,b.upper);
}
inline void SameWeights(const sc::NodalWallWeights& a,const sc::NodalWallWeights& b) {
  ASSERT_TRUE(a.prepared());ASSERT_TRUE(b.prepared());
  ASSERT_EQ(a.global_node_count(),b.global_node_count());
  ASSERT_EQ(a.parent_count(),b.parent_count());ASSERT_EQ(a.node_count(),b.node_count());
  Same(a.total_area(),b.total_area());
  for(unsigned p=0;p<a.parent_count();++p) {
    const auto& x=a.parent(p);const auto& y=b.parent(p);
    EXPECT_EQ(x.parent_element_id,y.parent_element_id);EXPECT_EQ(x.feature_id,y.feature_id);
    EXPECT_EQ(x.parent_face_id,y.parent_face_id);EXPECT_EQ(x.family,y.family);EXPECT_EQ(x.arity,y.arity);
    for(unsigned i=0;i<4;++i) {
      EXPECT_EQ(x.nodes[i],y.nodes[i]);
    }
    Same(x.area,y.area);Same(x.share,y.share);
  }
  for(unsigned n=0;n<a.node_count();++n) {EXPECT_EQ(a.node(n).node,b.node(n).node);Same(a.node(n).area,b.node(n).area);}
}
struct TriangleSource {
  TriangleSource(std::uint32_t n,std::uint32_t p):nodes(n),positions(3*n),references(p),input(p) {}
  std::uint32_t nodes;
  std::vector<double> positions;
  std::vector<sc::T3MaterialMeasure> references;
  std::vector<sc::NodalWallParentInput> input;
  static constexpr std::uint64_t FirstId=UINT64_C(1)<<54;
  sc::VectorView View() const {return {positions.data(),nodes,3,1};}
  bool Prepare() {
    for(std::uint32_t n=0;n<nodes;++n) {
      positions[3*n]=2.*(n/3)+(n%3==1?1.:0.);positions[3*n+1]=n%3==2?1.:0.;
    }
    for(std::size_t p=0;p<input.size();++p) {
      const auto n=std::min<std::uint32_t>(3*(p%((nodes+2)/3)),nodes-3);
      sc::SurfaceTriangle parent;
      parent.nodes[0]=n;parent.nodes[1]=n+1;parent.nodes[2]=n+2;
      parent.parent_element_id=FirstId+2*p+1;parent.feature_id=FirstId+2*p+2;
      parent.interpolation=sc::SurfaceInterpolation::kLinearTriangle;
      if(sc::PrepareT3MaterialMeasure(View(),parent,&references[p])!=sc::SurfaceMeasureStatus::Ok)return false;
      input[p]={nullptr,0,&references[p]};
    }
    return true;
  }
};
} // namespace vehicle_wall_weight_test
