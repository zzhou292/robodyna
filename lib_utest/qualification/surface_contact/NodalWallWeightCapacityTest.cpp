#include "NodalWallOwnerFixture.h"
#include <vector>

namespace {
namespace sc=tlfea::contact;
using nodal_wall_owner_test::Bytes;
using nodal_wall_owner_test::Same;
constexpr unsigned Nodes=1030,Quads=804,Triangles=111,Parents=Quads+Triangles;
constexpr std::uint64_t FirstId=std::uint64_t{1}<<54;
constexpr long double SquareArea=1.L/64;
struct Source {
  std::vector<double> x=std::vector<double>(3*Nodes);
  std::vector<sc::Q4ParametricReference> q=std::vector<sc::Q4ParametricReference>(Quads);
  std::vector<sc::T3MaterialMeasure> t=std::vector<sc::T3MaterialMeasure>(Triangles);
  std::vector<sc::NodalWallParentInput> input=std::vector<sc::NodalWallParentInput>(Parents);
  sc::VectorView positions() const { return {x.data(),Nodes,3,1}; }
  bool Prepare() {
    for(unsigned n=0;n<Nodes;++n) { x[3*n+1]=(n%10)*.125; x[3*n+2]=(n/10)*.125; }
    for(unsigned p=0;p<Quads;++p) {
      const unsigned n=10*(p/9)+p%9;
      sc::SurfaceQ4 parent;
      parent.nodes[0]=n; parent.nodes[1]=n+1; parent.nodes[2]=n+11; parent.nodes[3]=n+10;
      parent.parent_element_id=FirstId+2*p+1; parent.feature_id=parent.parent_element_id+4096;
      if(q[p].Initialize(positions(),&parent,1).status!=sc::Q4ParametricStatus::Ok) return false;
      input[p]={&q[p],0,nullptr};
    }
    for(unsigned p=0;p<Triangles;++p) {
      // A separate tail strip includes the last global node. This is contact
      // startup coverage, not a claim that this synthetic mesh is connected.
      const unsigned cell=862+p/2,n=10*(cell/9)+cell%9;
      sc::SurfaceTriangle parent;
      parent.nodes[0]=n; parent.nodes[1]=p%2?n+11:n+1; parent.nodes[2]=p%2?n+10:n+11;
      parent.parent_element_id=FirstId+2*(Quads+p)+1; parent.feature_id=parent.parent_element_id+4096;
      parent.interpolation=sc::SurfaceInterpolation::kLinearTriangle;
      if(sc::PrepareT3MaterialMeasure(positions(),parent,&t[p])!=sc::SurfaceMeasureStatus::Ok) return false;
      input[Quads+p]={nullptr,0,&t[p]};
    }
    return true;
  }
};
void SameWeights(const sc::NodalWallWeights& a,const sc::NodalWallWeights& b) {
  ASSERT_TRUE(a.prepared()); ASSERT_TRUE(b.prepared());
  ASSERT_EQ(a.global_node_count(),b.global_node_count());
  ASSERT_EQ(a.parent_count(),b.parent_count()); ASSERT_EQ(a.node_count(),b.node_count());
  Same(a.total_area(),b.total_area());
  for(unsigned p=0;p<a.parent_count();++p) {
    const auto& x=a.parent(p); const auto& y=b.parent(p);
    EXPECT_EQ(x.parent_element_id,y.parent_element_id); EXPECT_EQ(x.feature_id,y.feature_id);
    EXPECT_EQ(x.parent_face_id,y.parent_face_id); EXPECT_EQ(x.arity,y.arity); EXPECT_EQ(x.family,y.family);
    for(unsigned local=0;local<4;++local) EXPECT_EQ(x.nodes[local],y.nodes[local]);
    Same(x.area,y.area); Same(x.share,y.share);
  }
  for(unsigned n=0;n<a.node_count();++n) {
    EXPECT_EQ(a.node(n).node,b.node(n).node); Same(a.node(n).area,b.node(n).area);
  }
}
TEST(NodalWallWeights, CompleteLargeMixedWeightsRetainAreaOrderAndOwnedLifetime) {
  sc::NodalWallWeights kept;
  {
    auto source=std::make_unique<Source>(); ASSERT_TRUE(source->Prepare());
    ASSERT_EQ(kept.Initialize(Nodes,source->input.data(),Parents,{}).status,sc::NodalWallStatus::Ok);
    EXPECT_EQ(kept.parent_count(),Parents); EXPECT_EQ(kept.global_node_count(),Nodes);
    ASSERT_GT(kept.node_count(),128u); EXPECT_EQ(kept.node(kept.node_count()-1).node,Nodes-1);
    std::vector<long double> area(Nodes);
    for(unsigned p=0;p<Parents;++p) {
      const auto& parent=kept.parent(p); const long double a=p<Quads?SquareArea:SquareArea/2;
      EXPECT_EQ(parent.parent_element_id,FirstId+2*p+1);
      EXPECT_LE(parent.area.lower,a); EXPECT_GE(parent.area.upper,a);
      EXPECT_EQ(parent.arity,p<Quads?4u:3u);
      for(unsigned local=0;local<parent.arity;++local) area[parent.nodes[local]]+=a/parent.arity;
    }
    long double sum=0;
    for(unsigned n=0;n<kept.node_count();++n) {
      const auto& actual=kept.node(n); const auto expected=area[actual.node];
      EXPECT_LE(actual.area.lower,expected); EXPECT_GE(actual.area.upper,expected); sum+=expected;
    }
    const auto exact=(Quads+Triangles/2.L)*SquareArea;
    EXPECT_LE(std::abs(sum-exact),1e-14L*exact);
    EXPECT_LE(kept.total_area().lower,exact); EXPECT_GE(kept.total_area().upper,exact);
    std::reverse(source->input.begin(),source->input.end()); sc::NodalWallWeights reversed;
    ASSERT_EQ(reversed.Initialize(Nodes,source->input.data(),Parents,{}).status,sc::NodalWallStatus::Ok);
    SameWeights(kept,reversed);
  }
  sc::NodalWallWeights copied(kept),moved(std::move(copied)),assigned;
  assigned=std::move(moved); SameWeights(kept,copied); SameWeights(kept,moved); SameWeights(kept,assigned);
  EXPECT_GT(kept.owned_payload_bytes(),sizeof(kept));
}
TEST(NodalWallWeights, CountAndByteLimitsRejectBeforeBorrowedReadsAndLateFailurePreservesCopy) {
  auto source=std::make_unique<Source>(); ASSERT_TRUE(source->Prepare()); sc::NodalWallWeights value;
  ASSERT_EQ(value.Initialize(Nodes,source->input.data(),Parents,{}).status,sc::NodalWallStatus::Ok);
  const sc::NodalWallWeights held(value); const auto bytes=Bytes(value);
  const auto* unreadable=reinterpret_cast<const sc::NodalWallParentInput*>(std::uintptr_t{1});
  EXPECT_EQ(value.Initialize(Nodes,unreadable,Parents).status,sc::NodalWallStatus::Capacity);
  sc::NodalWallWeightLimits cap; cap.max_owned_bytes=value.owned_payload_bytes()-1;
  EXPECT_EQ(value.Initialize(Nodes,unreadable,Parents,cap).status,sc::NodalWallStatus::Capacity);
  cap={}; cap.max_parents=Parents-1;
  EXPECT_EQ(value.Initialize(Nodes,unreadable,Parents,cap).status,sc::NodalWallStatus::Capacity);
  cap={}; cap.max_nodes=Nodes-1;
  EXPECT_EQ(value.Initialize(Nodes,unreadable,Parents,cap).status,sc::NodalWallStatus::Capacity);
  cap={}; cap.max_nodes=sc::MaxNodalWallWeightNodes+1;
  EXPECT_EQ(value.Initialize(Nodes,unreadable,Parents,cap).status,sc::NodalWallStatus::InvalidInput);
  EXPECT_EQ(Bytes(value),bytes); SameWeights(value,held);
  const auto last=source->input.back(); source->input.back()={};
  const auto report=value.Initialize(Nodes,source->input.data(),Parents,{});
  EXPECT_EQ(report.status,sc::NodalWallStatus::InvalidReference); EXPECT_EQ(report.parent,Parents-1);
  EXPECT_EQ(Bytes(value),bytes); SameWeights(value,held);
  source->input.back()=last; cap={}; cap.max_owned_bytes=value.owned_payload_bytes();
  ASSERT_EQ(value.Initialize(Nodes,source->input.data(),Parents,cap).status,sc::NodalWallStatus::Ok);
  SameWeights(value,held);
}
TEST(NodalWallWeights, LargeStartupCannotEnterLegacyResultOrResidentStorage) {
  auto source=std::make_unique<Source>(); ASSERT_TRUE(source->Prepare()); sc::NodalWallWeights weights;
  ASSERT_EQ(weights.Initialize(Nodes,source->input.data(),Parents,{}).status,sc::NodalWallStatus::Ok);
  // These views advertise the right size but only have one readable scalar;
  // capacity must reject before any position, mass or result-array indexing.
  double scalar=0; std::uint8_t fixed=0;
  sc::VectorView view{&scalar,Nodes,3,1};
  sc::NodalWallResult result; result.attempt=918; const auto old=Bytes(result);
  EXPECT_EQ(sc::EvaluateNodalWallContact(weights,view,view,
      {&scalar,&fixed,Nodes,0,sc::TranslationMassModel::kIsotropicLumped},
      {0,16,.5,1e-6,1e-6},1,&result).status,sc::NodalWallStatus::Capacity);
  EXPECT_EQ(Bytes(result),old);
  nodal_wall_owner_test::Fixture f; ASSERT_TRUE(f.Prepare()); auto config=f.Config(); config.owner.node_count=Nodes;
  auto model=std::make_unique<sc::nodal_wall_device_detail::Model>();
  EXPECT_EQ(sc::nodal_wall_device_detail::PrepareModel(config,f.wall.view(),weights,view,&scalar,&fixed,f.motion,model.get()).status,
      sc::NodalWallDeviceStatus::ResourceLimit);
  EXPECT_FALSE(model->prepared);
}
} // namespace
