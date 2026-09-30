#pragma once
// Synthetic source-sized host collection; no source-part authentication claim.
#include "ShellBatchBindingFixture.h"
#include <algorithm>

namespace shell_collection_test {
using namespace shell_binding_test;
constexpr std::size_t QCount=88,TCount=6,NodeCount=117;
constexpr std::uint64_t ParentBase=std::uint64_t{1}<<50;
struct Fixture {
  std::array<fe::ShellQephBindingInput,fe::MaxShellCollectionParents> q{};
  std::array<fe::ShellT3BindingInput,fe::MaxShellCollectionParents> t{};
  std::size_t q_count=0,t_count=0,node_count=0;
  fe::ShellBatchCollectionInput input() const {
    return {q_count?q.data():nullptr,t_count?t.data():nullptr,q_count,t_count,node_count};
  }
};
inline std::uint64_t NodeId(std::size_t n) { return n<108?1000+n:WideId+n; }
inline tl::math::Vec3 Grid(unsigned column,unsigned row) {
  return {column+column*column/32.,row+row*row/64.,0};
}
template<class Input,std::size_t N>
void Set(Input& in,const std::array<std::size_t,N>& nodes,
         const std::array<tl::math::Vec3,fe::MaxShellCollectionNodes>& positions,unsigned parent) {
  in.density=1024+8*parent; in.thickness=(16+parent%5)/1024.;
  in.young_modulus=2e6*(1+(parent%7)/8.); in.poisson_ratio=.25+(parent%3)/32.;
  for(std::size_t i=0;i<N;++i) {
    in.node_ids[i]=static_cast<decltype(in.node_ids[0]+0)>(NodeId(nodes[i]));
    in.position[i]=positions[nodes[i]];
  }
}
inline Fixture Connected() {
  Fixture f; f.q_count=QCount; f.t_count=TCount; f.node_count=NodeCount;
  std::array<tl::math::Vec3,fe::MaxShellCollectionNodes> positions{};
  for(unsigned row=0;row<9;++row) for(unsigned col=0;col<12;++col)
    positions[12*row+col]=Grid(col,row);
  for(unsigned row=0;row<8;++row) for(unsigned col=0;col<11;++col) {
    const unsigned i=11*row+col,n=12*row+col;
    f.q[i].nodes={n,n+1,n+13,n+12}; f.q[i].source_parent_id=ParentBase+i;
    Set(f.q[i].reference,f.q[i].nodes,positions,i);
  }
  // Three scalene wings: an edge-attached triangle, followed by a triangle
  // sharing its tip. The builder declares no attachment/overlap mechanics.
  for(unsigned wing=0;wing<3;++wing) {
    const unsigned a=24*wing+11,b=a+12,n=108+3*wing;
    const auto x=positions[a]; const double dy=positions[b].y-x.y;
    positions[n]={x.x+.75,x.y+dy/4,0};
    positions[n+1]={x.x+1.75,x.y-dy/8,0};
    positions[n+2]={x.x+1.25,x.y+dy,0};
    f.t[2*wing].nodes={a,n,b}; f.t[2*wing+1].nodes={n,n+1,n+2};
    for(unsigned k=0;k<2;++k) {
      const unsigned i=2*wing+k;
      f.t[i].source_parent_id=ParentBase+QCount+i;
      Set(f.t[i].reference,f.t[i].nodes,positions,QCount+i);
    }
  }
  return f;
}
inline Fixture Pair() {
  const auto old=Edge(); Fixture f; f.q_count=f.t_count=1; f.node_count=old.node_count;
  f.q[0]={old.qeph,old.qeph_nodes,ParentBase};
  f.t[0]={old.t3,old.t3_nodes,ParentBase+1}; return f;
}
inline void MassNear(const fe::ShellBindingMass& actual,const WideMass& expected) {
  Near(actual.mass,expected.mass); Near(actual.isotropic_inertia,expected.total);
  Near(actual.physical_inertia,expected.physical); Near(actual.added_inertia,expected.added);
}
inline void CheckAnalytic(const Fixture& f,const Binding& b) {
  std::array<WideMass,fe::MaxShellCollectionNodes> expected{};
  for(std::size_t i=0;i<f.q_count;++i) {
    const auto& in=f.q[i].reference; const auto& x=in.position;
    const long double area=(independent::CrossNorm(independent::Difference(x[1],x[0]),independent::Difference(x[2],x[0]))+
      independent::CrossNorm(independent::Difference(x[2],x[0]),independent::Difference(x[3],x[0])))/2;
    const long double m=static_cast<long double>(in.density)*in.thickness*area/4;
    const long double physical=m*in.thickness*in.thickness/12,added=m*area/12;
    for(auto n:f.q[i].nodes) Add(expected[n],{m,physical+added,physical,added});
  }
  for(std::size_t i=0;i<f.t_count;++i) {
    const auto truth=independent::Independent(NativeInput(f.t[i].reference));
    for(unsigned local=0;local<3;++local) {
      const auto w=truth.weight[local];
      Add(expected[f.t[i].nodes[local]],{w*truth.mass,w*truth.total,w*truth.physical,w*truth.added});
    }
  }
  WideMass total;
  for(std::size_t n=0;n<f.node_count;++n) {
    SCOPED_TRACE(n);
    MassNear(b.nodes()[n].native,expected[n]); Add(total,expected[n]);
  }
  MassNear(b.totals(),total);
}
inline void CheckNative(const Fixture& f,const Binding& b) {
  std::array<fe::ShellBindingMass,fe::MaxShellCollectionNodes> expected{};
  fe::ShellBindingMass total;
  for(std::size_t i=0;i<f.q_count;++i) {
    SCOPED_TRACE(i);
    fe::qeph::ReferenceData q;
    ASSERT_EQ(fe::qeph::InitializeReference(f.q[i].reference,q),fe::qeph::Status::kSuccess);
    EXPECT_EQ(b.qeph_nodes(i),f.q[i].nodes); EXPECT_EQ(b.qeph_source_id(i),f.q[i].source_parent_id);
    const auto& saved=b.qeph_reference(i);
    EXPECT_TRUE(saved.prepared); EXPECT_EQ(Bytes(saved.input),Bytes(q.input));
    for(unsigned local=0;local<4;++local) {
      const fe::ShellBindingMass m{q.nodal_mass[local],q.isotropic_inertia[local],q.physical_inertia[local],q.added_inertia[local]};
      AddExact(expected[f.q[i].nodes[local]],m); AddExact(total,m);
    }
  }
  for(std::size_t i=0;i<f.t_count;++i) {
    SCOPED_TRACE(i);
    fe::t3::ReferenceData t;
    ASSERT_EQ(fe::t3::InitializeReference(f.t[i].reference,t),fe::t3::Status::kSuccess);
    EXPECT_EQ(b.t3_nodes(i),f.t[i].nodes); EXPECT_EQ(b.t3_source_id(i),f.t[i].source_parent_id);
    const auto& saved=b.t3_reference(i);
    EXPECT_TRUE(saved.prepared); EXPECT_EQ(Bytes(saved.input),Bytes(t.input));
    for(unsigned local=0;local<3;++local) {
      const fe::ShellBindingMass m{t.nodal_mass[local],t.isotropic_inertia[local],t.physical_inertia[local],t.added_inertia[local]};
      AddExact(expected[f.t[i].nodes[local]],m); AddExact(total,m);
    }
  }
  for(std::size_t n=0;n<f.node_count;++n) Exact(b.nodes()[n].native,expected[n]);
  Exact(b.totals(),total);
}
inline void RejectRetry(const Fixture& bad,Status expected,
    fe::ShellBindingFamily family=fe::ShellBindingFamily::None,
    std::size_t parent=fe::NoShellBindingNode) {
  Binding b; const auto before=Bytes(b); const auto input_before=Bytes(bad);
  const auto report=b.Initialize(bad.input());
  EXPECT_EQ(report.status,expected)<<report.message;
  EXPECT_EQ(report.family,family); EXPECT_EQ(report.parent_index,parent);
  EXPECT_EQ(Bytes(b),before); EXPECT_EQ(Bytes(bad),input_before);
  const auto good=Connected();
  ASSERT_EQ(b.Initialize(good.input()).status,Status::Success);
  Binding clean;
  ASSERT_EQ(clean.Initialize(good.input()).status,Status::Success);
  EXPECT_EQ(b.inventory(),clean.inventory()); Exact(b.totals(),clean.totals());
}
} // namespace shell_collection_test
