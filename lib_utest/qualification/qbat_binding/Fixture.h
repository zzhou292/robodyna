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
// at the edge. The contact-only option adds a second distinct centered T3 for
// rigid self-contact fixtures. These are small synthetic layers, not a model.
struct Fixture {
  std::array<fe::ShellQephBindingInput,2> q;
  fe::ShellT3BindingInput t;
  std::array<fe::ShellT3BindingInput,2> contact_t;
  fe::ShellQbatBindingInput b;
  bool contact_geometry = false;
  bool distinct_contact_t3 = false;
  bool interior_edge_contact = false;
  explicit Fixture(bool contact = false, bool distinct_t3 = false,
                   bool interior_ee = false)
      : contact_geometry(contact || interior_ee),
        distinct_contact_t3(distinct_t3),
        interior_edge_contact(interior_ee) {
    const tl::math::Vec3 x[]{
        {0,0,0},{.04,0,0},{.04,.02,0},{0,.02,0},
        interior_ee ? tl::math::Vec3{-.01,.01,.00025}
                    : (contact ? tl::math::Vec3{.01,.003,.00025}
                               : tl::math::Vec3{.05,.01,0}),
        interior_ee ? tl::math::Vec3{.02,.01,.00025}
                    : tl::math::Vec3{.03,.003,.00025},
        interior_ee ? tl::math::Vec3{.01,.015,.00025}
                    : tl::math::Vec3{.02,.006,.00025}};
    for(unsigned layer=0;layer<2;++layer) {
      q[layer].source_parent_id=100+layer;
      q[layer].nodes={0,1,2,3};
      auto& input=q[layer].reference;
      input.density=2500;
      input.young_modulus=60e9;
      input.poisson_ratio=.25;
      input.thickness=.002;
      input.placement=layer?fe::ShellReferencePlacement::BottomReferencePlane:
          (contact_geometry ? fe::ShellReferencePlacement::Centered :
                     fe::ShellReferencePlacement::TopReferencePlane);
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
    t.nodes = contact_geometry
        ? std::array<std::size_t,3>{4,5,6}
        : std::array<std::size_t,3>{1,4,2};
    t.source_parent_id=102;
    t.reference.density=1000;
    t.reference.young_modulus=250e6;
    t.reference.poisson_ratio=.35;
    t.reference.thickness=.0005;
    for(unsigned n=0;n<3;++n) {
      t.reference.position[n]=x[t.nodes[n]];
      t.reference.node_ids[n] = contact_geometry
          ? 14 + n : 10 + t.nodes[n];
    }
    contact_t[0]=t;
    contact_t[1]=t;
    contact_t[1].source_parent_id=104;
    contact_t[1].nodes={7,8,9};
    const tl::math::Vec3 second[]{
        {.02,.004,.0005},{.03,.004,.02},{.02,.006,.02}};
    for (unsigned n=0;n<3;++n) {
      contact_t[1].reference.node_ids[n]=17+n;
      contact_t[1].reference.position[n]=second[n];
    }
  }
  fe::ShellFormulationCollectionInput Input() const {
    return {{q.data(),distinct_contact_t3 ? contact_t.data() : &t,
             2,distinct_contact_t3 ? 2u : 1u,
             distinct_contact_t3 ? 10u : (contact_geometry ? 7u : 5u)},
            &b,1};
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
