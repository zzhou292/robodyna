// SPDX-License-Identifier: MIT
#pragma once
#include "lib_src/elements/type13/resident/AssemblyValues.h"
#include "lib_src/elements/type25/mapped/AssemblyValues.h"
#include <gtest/gtest.h>
#include <array>
#include <algorithm>
#include <vector>
#include <type_traits>
#include <cstring>
#include <limits>
namespace connector_test {
namespace fe = tl::fea;
namespace mc = fe::mapped_connector;
namespace a = fe::type13::batch_detail;
namespace b = fe::type25::batch_detail;
constexpr std::size_t Parents = 5, Nodes = 139;
inline std::uint64_t Bits(double x) { std::uint64_t out; std::memcpy(&out, &x, sizeof(out)); return out; }
struct Inputs {
  double position[3*Nodes]{}, velocity[3*Nodes]{}, omega[3*Nodes]{}, orientation[4*Nodes]{};
  double inverse[Nodes]{}, value[8][Nodes]{};
  std::uint8_t fixed[Nodes]{}, rotation[Nodes]{};
  cudaStream_t stream=nullptr;
  fe::NodalAssemblyResult result;
  fe::stability::RowBounds bounds;
  void Reset() {
    result = {}; result.base_epoch = 1; result.attempt = 3;
    bounds = {}; bounds.base_epoch = 1; bounds.attempt = 3; bounds.initialized = bounds.valid = true;
    for (std::size_t node = 0; node < Nodes; ++node) {
      inverse[node] = node % 2 ? 1 : 0; orientation[4*node] = 1; rotation[node] = 1;
      for (unsigned c = 0; c < 8; ++c) value[c][node] = c < 6 ? -0. : (node+c)*.125;
    }
    value[0][0] = 0x1p54; value[2][5] = -.875; value[4][138] = -0.;
  }
  fe::NodalAssemblyView View() {
    fe::NodalAssemblyView view;
    view.accepted = {position,velocity,omega,Nodes,1,orientation};
    view.mass.inverse_mass = inverse; view.mass.fixed = fixed; view.mass.node_count = Nodes;
    view.inverse_inertia = inverse; view.translation_fixed_bits = fixed; view.rotation_fixed = fixed;
    view.rotation_present = rotation;
    view.forces = {value[0],value[1],value[2],value[3],value[4],value[5],Nodes,1};
    view.result = &result; view.bounds = &bounds; view.attempt = 3; view.stream=stream; return view;
  }
  fe::NodalCinAssemblyView Cin() {
    fe::NodalCinAssemblyView cin; cin.translational_stiffness=value[6]; cin.rotational_stiffness=value[7];
    cin.node_count=Nodes; return cin;
  }
};
template<class Storage, class Element, class Evaluation, class Property>
struct Packet {
  Storage storage;
  Element elements[Parents]{};
  Evaluation values[Parents]{};
  Property properties[1]{};
  fe::type25::batch_detail::DeviceNode source_nodes[Nodes]{};
  std::uint32_t offsets[Nodes+1]{}, incidence[2*Parents]{}, touched[2*Parents]{};
  mc::Parent parent[Parents]{};
  fe::mapped_shell::AssemblyNode sums[2*Parents]{};
  unsigned long long failure=0;
  Inputs input;
  void Bind() {
    storage.model.config.owner.node_count=Nodes;
    storage.model.elements=elements; storage.model.properties=properties;
    storage.assembly={offsets,incidence,touched,0,parent,sums,&failure};
    const std::size_t connections[Parents][2]{{0,5},{3,0},{0,4},{3,4},{5,0}};
    for (unsigned p=0;p<Parents;++p) for (unsigned s=0;s<2;++s) elements[p].nodes[s]=connections[p][s];
    EXPECT_TRUE(fe::mapped_shell::BuildIncidence<2>(elements,Parents,Nodes,offsets,Nodes+1,incidence,2*Parents));
    for (std::uint32_t n=0;n<Nodes;++n) if (offsets[n]!=offsets[n+1]) touched[storage.assembly.touched_count++]=n;
    input.Reset();
    for (unsigned p=0;p<Parents;++p) for (unsigned s=0;s<2;++s) {
      const double x=p==0?-0x1p54:p==1?1:p==2?0x1p54:p==3?.375:-0x1p54;
      values[p].endpoints[s].force_N={x,-0.,.125*(p+s)};
      values[p].endpoints[s].couple_Nm={-0.,0.,-.0625*(p+1)};
    }
  }
};
using Packet13=Packet<a::Storage,a::DeviceElement,fe::type13::Evaluation,fe::type13::Property>;
using Packet25=Packet<b::Storage,b::DeviceElement,fe::type25::Evaluation,fe::type25::Property>;
inline void Prepare(Packet13& p) {
  p=Packet13{}; p.Bind(); p.storage.model.element_count=Parents;
  p.storage.model.config.assembly=fe::type13::BatchAssembly::CinNativeStiffness;
  p.storage.slab[0]=p.values;
  for (unsigned i=0;i<Parents;++i) p.values[i].stability={1e-6,1.25*(i+1),.125*(i+1)};
}
inline void Prepare(Packet25& p) {
  p=Packet25{}; p.Bind(); p.storage.model.config.element_count=Parents;
  p.storage.model.nodes=p.source_nodes; p.storage.slab[0].element=p.values;
  for (unsigned i=0;i<Parents;++i) {
    p.values[i].translation_stiffness_N_per_m=1.25*(i+1);
    p.values[i].rotation_stiffness_Nm_per_rad=.125*(i+1);
  }
}
inline void Remove(Packet13& p,unsigned parent) { p.values[parent].native_history.active=false; }
inline void Remove(Packet25& p,unsigned parent) { p.values[parent].history.active=false; }
inline void LargeCoefficient(Packet13& p,unsigned parent) { p.values[parent].stability.translation_stiffness_N_per_m=std::numeric_limits<double>::max(); }
inline void LargeCoefficient(Packet25& p,unsigned parent) { p.values[parent].translation_stiffness_N_per_m=std::numeric_limits<double>::max(); }
inline void BadCoefficient(Packet13& p,unsigned parent) { p.values[parent].stability.translation_stiffness_N_per_m=-1; }
inline void BadCoefficient(Packet25& p,unsigned parent) { p.values[parent].translation_stiffness_N_per_m=-1; }
inline void CompareValues(const Inputs& actual,const Inputs& expected) {
  for (unsigned c=0;c<8;++c) for (unsigned n=0;n<Nodes;++n)
    EXPECT_EQ(Bits(actual.value[c][n]),Bits(expected.value[c][n]))<<c<<":"<<n;
}
template<class Family,class P>
inline unsigned long long StageHost(P& p) {
  auto& state=p.storage; const auto view=p.input.View();const auto cin=p.input.Cin();
  const auto accepted=[](auto& s) {
    if constexpr(std::is_same_v<P,Packet13>) return 0u;
    else return &s.slab[0];
  }(state);
  auto failed=fe::mapped_shell::NoAssemblyFailure;
  for(unsigned parent=0;parent<Parents;++parent) {
    const auto record=Family::Prepare(state,accepted,parent,view,cin,false);
    state.assembly.parent[parent]=record;
    if(record.failure!=mc::Failure::None) failed=std::min(failed,mc::ParentKey(Family::Ordering,Parents,parent,record.failure));
  }
  for(std::size_t row=0;row<state.assembly.touched_count;++row) {
    const auto node=state.assembly.touched_nodes[row];
    const auto parent=mc::GatherNode(node,state.assembly,Family::Values(state,accepted),view.forces,
        cin.translational_stiffness,cin.rotational_stiffness,state.assembly.node[row]);
    if(parent!=UINT32_MAX) failed=std::min(failed,mc::AdditionKey(Family::Ordering,Parents,parent));
  }
  return failed;
}
} // namespace connector_test
