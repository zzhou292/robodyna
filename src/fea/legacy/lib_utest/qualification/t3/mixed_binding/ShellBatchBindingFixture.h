#pragma once
// Host-only immutable startup tests; no owner, native execution or dynamics.
#include "lib_src/elements/ShellBatchBinding.h"
#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_src/elements/t3/T3Startup.h"
#include "../../native/t3/T3StartupTestOracle.h"
#include <cstring>
#include <limits>

namespace shell_binding_test {
namespace fe=tl::fea;
namespace independent=tl::qualification::t3::test;
using Input=fe::ShellBatchBindingInput;
using Binding=fe::ShellBatchBinding;
using Status=fe::ShellBindingStatus;
constexpr std::uint64_t WideId=(std::uint64_t{1}<<54)+103;
constexpr long double Budget=2e-12L; // Existing startup dimensional tolerance.

template<class T> std::array<unsigned char,sizeof(T)> Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> result{};
  std::memcpy(result.data(),&value,sizeof(T)); return result;
}
inline Input Edge() {
  Input in;
  in.node_count=5; in.qeph_nodes={0,1,2,3}; in.t3_nodes={1,4,2};
  const tl::math::Vec3 x[5]{{0,0,0},{1,0,0},{1,1,0},{0,1,0},{1.75,.25,0}};
  in.qeph.density=in.t3.density=1024;
  in.qeph.thickness=in.t3.thickness=1./32;
  in.qeph.young_modulus=in.t3.young_modulus=2e6;
  in.qeph.poisson_ratio=in.t3.poisson_ratio=.3;
  for(unsigned i=0;i<4;++i) { in.qeph.position[i]=x[i]; in.qeph.node_ids[i]=100+i; }
  for(unsigned i=0;i<3;++i) {
    in.t3.position[i]=x[in.t3_nodes[i]];
    in.t3.node_ids[i]=in.t3_nodes[i]==4?WideId:100+in.t3_nodes[i];
  }
  return in;
}
inline Input Overlap(unsigned shared) {
  auto in=Edge();
  if(shared==0) {
    in.node_count=7; in.t3_nodes={4,5,6};
    for(unsigned i=0;i<3;++i) { in.t3.node_ids[i]=WideId+i; in.t3.position[i].x+=2; }
  } else if(shared==1) {
    in.node_count=6; in.t3_nodes={1,4,5};
    in.t3.node_ids[2]=WideId+1; in.t3.position[2]={1.5,1,0};
  } else if(shared==3) {
    in.node_count=4; in.t3_nodes={0,1,2};
    for(unsigned i=0;i<3;++i) { in.t3.position[i]=in.qeph.position[i]; in.t3.node_ids[i]=in.qeph.node_ids[i]; }
  }
  return in;
}
inline tl::qualification::t3::ReferenceInput NativeInput(const fe::t3::ReferenceInput& in) {
  tl::qualification::t3::ReferenceInput result;
  for(unsigned i=0;i<3;++i) { result.position[i]=in.position[i]; result.node_ids[i]=in.node_ids[i]; }
  result.density=in.density; result.thickness=in.thickness;
  result.young_modulus=in.young_modulus; result.poisson_ratio=in.poisson_ratio;
  return result;
}
struct WideMass { long double mass=0,total=0,physical=0,added=0; };
inline void Add(WideMass& a,const WideMass& b) {
  a.mass+=b.mass; a.total+=b.total; a.physical+=b.physical; a.added+=b.added;
}
inline std::array<WideMass,fe::MaxShellBindingNodes> Truth(const Input& in) {
  std::array<WideMass,fe::MaxShellBindingNodes> result{};
  // All Q4 fixtures here are flat convex rectangles. Two world-triangle areas
  // independently determine their physical area, without native frame code.
  const auto& x=in.qeph.position;
  const long double area=(independent::CrossNorm(independent::Difference(x[1],x[0]),independent::Difference(x[2],x[0]))+
      independent::CrossNorm(independent::Difference(x[2],x[0]),independent::Difference(x[3],x[0])))/2;
  const long double mass=static_cast<long double>(in.qeph.density)*in.qeph.thickness*area/4;
  const long double physical=mass*in.qeph.thickness*in.qeph.thickness/12,added=mass*area/12;
  for(auto n:in.qeph_nodes) Add(result[n],{mass,physical+added,physical,added});
  const auto tri=independent::Independent(NativeInput(in.t3));
  for(unsigned i=0;i<3;++i) {
    const auto p=tri.weight[i];
    Add(result[in.t3_nodes[i]],{tri.mass*p,tri.total*p,tri.physical*p,tri.added*p});
  }
  return result;
}
inline void Near(double actual,long double expected) {
  EXPECT_LE(std::abs(static_cast<long double>(actual)-expected),Budget*2*std::abs(expected));
}
inline void CheckTruth(const Input& in,const Binding& b) {
  ASSERT_TRUE(b.prepared()); ASSERT_EQ(b.node_count(),in.node_count);
  const auto expected=Truth(in); WideMass totals;
  for(unsigned n=0;n<in.node_count;++n) {
    SCOPED_TRACE(n);
    const auto& value=b.nodes()[n].native; const auto& e=expected[n];
    Near(value.mass,e.mass); Near(value.isotropic_inertia,e.total);
    Near(value.physical_inertia,e.physical); Near(value.added_inertia,e.added);
    EXPECT_GT(value.mass,0); EXPECT_GT(value.isotropic_inertia,0); Add(totals,e);
  }
  Near(b.totals().mass,totals.mass); Near(b.totals().isotropic_inertia,totals.total);
  Near(b.totals().physical_inertia,totals.physical); Near(b.totals().added_inertia,totals.added);
}
inline void AddExact(fe::ShellBindingMass& sum,const fe::ShellBindingMass& value) {
  sum.mass+=value.mass; sum.isotropic_inertia+=value.isotropic_inertia;
  sum.physical_inertia+=value.physical_inertia; sum.added_inertia+=value.added_inertia;
}
inline void Exact(const fe::ShellBindingMass& a,const fe::ShellBindingMass& b) {
  EXPECT_EQ(Bytes(a.mass),Bytes(b.mass)); EXPECT_EQ(Bytes(a.isotropic_inertia),Bytes(b.isotropic_inertia));
  EXPECT_EQ(Bytes(a.physical_inertia),Bytes(b.physical_inertia)); EXPECT_EQ(Bytes(a.added_inertia),Bytes(b.added_inertia));
}
inline void CheckNativeReduction(const Input& in,const Binding& binding) {
  fe::qeph::ReferenceData q; fe::t3::ReferenceData t;
  ASSERT_EQ(fe::qeph::InitializeReference(in.qeph,q),fe::qeph::Status::kSuccess);
  ASSERT_EQ(fe::t3::InitializeReference(in.t3,t),fe::t3::Status::kSuccess);
  std::array<fe::ShellBindingMass,fe::MaxShellBindingNodes> nodes{}; fe::ShellBindingMass total;
  for(unsigned i=0;i<4;++i) {
    const fe::ShellBindingMass value{q.nodal_mass[i],q.isotropic_inertia[i],q.physical_inertia[i],q.added_inertia[i]};
    AddExact(nodes[in.qeph_nodes[i]],value); AddExact(total,value);
  }
  for(unsigned i=0;i<3;++i) {
    const fe::ShellBindingMass value{t.nodal_mass[i],t.isotropic_inertia[i],t.physical_inertia[i],t.added_inertia[i]};
    AddExact(nodes[in.t3_nodes[i]],value); AddExact(total,value);
  }
  for(unsigned n=0;n<in.node_count;++n) Exact(nodes[n],binding.nodes()[n].native);
  Exact(total,binding.totals());
}
inline void Rejected(const Input& bad,Status status,fe::ShellBindingFamily family=fe::ShellBindingFamily::None) {
  Binding binding; const auto before=Bytes(binding); const auto input_before=Bytes(bad);
  const auto report=binding.Initialize(bad);
  EXPECT_EQ(report.status,status)<<report.message;
  if(family!=fe::ShellBindingFamily::None) EXPECT_EQ(report.family,family);
  EXPECT_EQ(Bytes(binding),before); EXPECT_EQ(Bytes(bad),input_before); EXPECT_FALSE(binding.prepared());
  const auto good=Edge(); ASSERT_EQ(binding.Initialize(good).status,Status::Success);
  Binding clean; ASSERT_EQ(clean.Initialize(good).status,Status::Success);
  EXPECT_EQ(binding.inventory(),clean.inventory());
  for(unsigned n=0;n<good.node_count;++n) Exact(binding.nodes()[n].native,clean.nodes()[n].native);
}
} // namespace shell_binding_test
