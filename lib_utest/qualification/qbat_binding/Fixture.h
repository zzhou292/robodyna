// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/ShellBatchBinding.h"
#include "lib_src/elements/qbat/QbatReference.h"
#include "lib_src/elements/t3/T3Startup.h"
#include <gtest/gtest.h>
#include <cstring>
#include <vector>

namespace qbat_binding_test {
namespace fe=tl::fea;
using Binding=fe::ShellBatchBinding;
using Status=fe::ShellBindingStatus;
using Mass=fe::ShellBindingMass;
template<class T> std::array<unsigned char,sizeof(T)> Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> bytes{};
  std::memcpy(bytes.data(),&value,sizeof(value));
  return bytes;
}
inline std::uint64_t Bits(double value) {
  std::uint64_t result;
  std::memcpy(&result,&value,sizeof(value));
  return result;
}
inline void Exact(const Mass& a,const Mass& b) {
  EXPECT_EQ(Bits(a.mass),Bits(b.mass));
  EXPECT_EQ(Bits(a.isotropic_inertia),Bits(b.isotropic_inertia));
  EXPECT_EQ(Bits(a.physical_inertia),Bits(b.physical_inertia));
  EXPECT_EQ(Bits(a.added_inertia),Bits(b.added_inertia));
}
inline void Add(Mass& a,const Mass& b) {
  a.mass+=b.mass;
  a.isotropic_inertia+=b.isotropic_inertia;
  a.physical_inertia+=b.physical_inertia;
  a.added_inertia+=b.added_inertia;
}
template<class R> Mass Term(const R& reference,unsigned n) {
  return {reference.nodal_mass[n],reference.isotropic_inertia[n],
      reference.physical_inertia[n],reference.added_inertia[n]};
}
// Three physical layers use one shared Q4 node set. A separate T3 contributes
// at the edge. These are explicit small synthetic layers, not a source model.
struct Fixture {
  std::array<fe::ShellQephBindingInput,2> q;
  fe::ShellT3BindingInput t;
  fe::ShellQbatBindingInput b;
  bool contact_geometry = false;
  explicit Fixture(bool contact = false) : contact_geometry(contact) {
    const tl::math::Vec3 x[]{
        {0,0,0},{.04,0,0},{.04,.02,0},{0,.02,0},
        contact ? tl::math::Vec3{.01,.003,.00025}
                         : tl::math::Vec3{.05,.01,0},
        {.03,.003,.00025},{.02,.006,.00025}};
    for(unsigned layer=0;layer<2;++layer) {
      q[layer].source_parent_id=100+layer;
      q[layer].nodes={0,1,2,3};
      auto& input=q[layer].reference;
      input.density=2500;
      input.young_modulus=60e9;
      input.poisson_ratio=.25;
      input.thickness=.002;
      input.placement=layer?fe::ShellReferencePlacement::BottomReferencePlane:
          fe::ShellReferencePlacement::TopReferencePlane;
      for(unsigned n=0;n<4;++n) {
        input.position[n]=x[n];
        input.node_ids[n]=10+n;
      }
    }
    b.nodes={0,1,2,3};
    b.source_parent_id=103;
    b.reference.quadrilateral=q[0].reference;
    auto& membrane=b.reference.quadrilateral;
    membrane.placement=fe::ShellReferencePlacement::Centered;
    membrane.density=1000;
    membrane.young_modulus=250e6;
    membrane.poisson_ratio=.35;
    membrane.thickness=.0005;
    b.reference.initial_a11_pa=membrane.young_modulus/(1-membrane.poisson_ratio*membrane.poisson_ratio);
    t.nodes = contact
        ? std::array<std::size_t,3>{4,5,6}
        : std::array<std::size_t,3>{1,4,2};
    t.source_parent_id=102;
    t.reference.density=1000;
    t.reference.young_modulus=250e6;
    t.reference.poisson_ratio=.35;
    t.reference.thickness=.0005;
    for(unsigned n=0;n<3;++n) {
      t.reference.position[n]=x[t.nodes[n]];
      t.reference.node_ids[n] = contact
          ? 14 + n : 10 + t.nodes[n];
    }
  }
  fe::ShellFormulationCollectionInput Input() const {
    return {{q.data(),&t,2,1,contact_geometry ? 7u : 5u},&b,1};
  }
};
inline void Reduction(const Binding& binding) {
  std::vector<Mass> nodes(binding.node_count());
  Mass total,sub;
  auto accumulate=[&](const auto& reference,const auto& indices,bool qbat) {
    for(unsigned n=0;n<indices.size();++n) {
      const auto term=Term(reference,n);
      Add(nodes[indices[n]],term);
      Add(total,term);
      if(qbat) Add(sub,term);
    }
  };
  for(std::size_t i=0;i<binding.qeph_count();++i)
    accumulate(binding.qeph_reference(i),binding.qeph_nodes(i),false);
  for(std::size_t i=0;i<binding.t3_count();++i)
    accumulate(binding.t3_reference(i),binding.t3_nodes(i),false);
  for(std::size_t i=0;i<binding.qbat_count();++i)
    accumulate(binding.qbat_reference(i).quadrilateral(),binding.qbat_nodes(i),true);
  for(std::size_t n=0;n<nodes.size();++n) Exact(binding.nodes()[n].native,nodes[n]);
  Exact(binding.totals(),total);
  Exact(binding.qbat_totals(),sub);
}
} // namespace qbat_binding_test
