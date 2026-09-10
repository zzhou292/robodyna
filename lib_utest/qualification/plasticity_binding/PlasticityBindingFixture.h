#pragma once
#include "lib_src/elements/ShellBatchPlasticityBinding.h"
#include <gtest/gtest.h>
#include <cstring>

namespace plasticity_binding_test {
namespace fe=tl::fea;
using Status=fe::ShellPlasticityBindingStatus;
using Binding=fe::ShellBatchPlasticityBinding;
template<class T> std::array<unsigned char,sizeof(T)> Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> bytes{}; std::memcpy(bytes.data(),&value,sizeof value); return bytes;
}
struct Fixture {
  fe::ShellBatchBindingInput geometry;
  fe::ShellQephBindingInput qeph;
  fe::ShellT3BindingInput t3;
  double x[3]{0,.1,.3},yq[3]{2700,3400,3620},yt[3]{5400,6800,7240};
  std::array<fe::ShellPlasticityCurveInput,2> curves;
  std::array<fe::ShellPlasticityMaterialInput,2> materials;
  std::array<fe::ShellPlasticitySectionInput,2> sections;
  std::array<fe::ShellPlasticityParentInput,2> parents;
  Fixture() {
    geometry.node_count=5; geometry.qeph_nodes={0,1,2,3}; geometry.t3_nodes={1,4,2};
    const tl::math::Vec3 nodes[5]{{0,0,0},{1,0,0},{1,1,0},{0,1,0},{1.75,.25,0}};
    geometry.qeph.young_modulus=2e6; geometry.qeph.poisson_ratio=.3;
    geometry.qeph.density=1024; geometry.qeph.thickness=1./32;
    geometry.t3.young_modulus=3e6; geometry.t3.poisson_ratio=.25;
    geometry.t3.density=768; geometry.t3.thickness=1./64;
    for(unsigned n=0;n<4;++n) { geometry.qeph.position[n]=nodes[n]; geometry.qeph.node_ids[n]=100+n; }
    for(unsigned n=0;n<3;++n) { const auto global=geometry.t3_nodes[n];
      geometry.t3.position[n]=nodes[global]; geometry.t3.node_ids[n]=100+global; }
    qeph={geometry.qeph,geometry.qeph_nodes,701}; t3={geometry.t3,geometry.t3_nodes,702};
    curves={{{47,{x,yq,3}},{48,{x,yt,3}}}};
    materials={{{37,47,2e6,.3,1024,{}},{38,48,3e6,.25,768,{true,8000,8,10000}}}};
    sections={{{57,1./32,3},{58,1./64,3}}};
    // Deliberately retain source order T3 then QEPH, independently of family order.
    parents={{{fe::ShellBindingFamily::T3,0,702,82,38,58},
              {fe::ShellBindingFamily::Qeph,0,701,81,37,57}}};
  }
  fe::ShellBatchCollectionInput collection() const { return {&qeph,&t3,1,1,5}; }
  fe::ShellBatchPlasticityBindingInput catalog() const {
    return {curves.data(),materials.data(),sections.data(),parents.data(),2,2,2,2};
  }
  void Translate(double dx) {
    for(auto& p:qeph.reference.position) p.x+=dx;
    for(auto& p:t3.reference.position) p.x+=dx;
  }
};
} // namespace plasticity_binding_test
