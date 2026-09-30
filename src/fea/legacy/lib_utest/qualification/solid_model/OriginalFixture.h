// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
#include "../solid18_reference/SourceFixture.h"
#include "../solid24_reference/SourceFixture.h"
#include "../solid24_reference/JacobianSupport.h"
#include "../solid6z_reference/SourceFixture.h"
#include "../solid_law36_point/source_fixture/OriginalMaterial.h"
#include <map>

namespace solid_model_test {
// Shared immutable qualification input; never an application source parser.
struct OriginalFixture {
  std::vector<s::Input18> a;
  std::vector<s::Input24> b;
  std::vector<s::Input6z> c;
  std::vector<fe::NodalDomainNode> nodes;
  fe::NodalNodeDomain domain;
  s::Model model;
  OriginalFixture() {
    std::map<std::uint64_t,tl::math::Vec3> unique;
    const auto collect=[&](const auto& input,unsigned arity) {
      for (unsigned n=0;n<arity;++n) {
        const auto added=unique.emplace(input.source_node_id[n],input.position_m[n]);
        if (!added.second) {
          const auto x=added.first->second,y=input.position_m[n];
          Require(Bits(x.x,y.x)&&Bits(x.y,y.y)&&Bits(x.z,y.z));
        }
      }
    };
    for (unsigned i=0;i<solid18_test::SourceCount;++i) {
      const auto input=solid18_test::Source(i);
      collect(input,8);
      s::Input18 parent;
      Require(fe::solid18::InitializeReference(input,parent.reference)==fe::solid18::Status::Success);
      Require(tl::material::law36::Prepare(law36_test::E,law36_test::Nu,input.density_kg_m3,
          law36_test::Curve,parent.material)==tl::material::law36::Status::Ok);
      a.push_back(parent);
    }
    for (unsigned i=0;i<solid24_test::SourceCount;++i) {
      auto input=solid24_test::Source(i);
      fe::solid24::Material material;
      Require(tl::material::law42::Prepare(24e6,.463,input.density_kg_m3,1e26,material)==
          tl::material::law42::Status::Ok);
      if (solid24_test::IsBrick(input)) {
        collect(input,8);
        input.profile.reference_strain=fe::solid24::ReferenceStrain::TotalLagrangian10;
        input.profile.working_length=fe::solid24::WorkingLengthUnit::Millimetre;
        s::Input24 parent;
        parent.material=material;
        Require(fe::solid24::InitializeReference(input,parent.reference)==fe::solid24::Status::Success);
        b.push_back(parent);
      } else {
        fe::solid6z::ReferenceInput wedge;
        Require(solid6z_test::Source(i,wedge));
        collect(wedge,6);
        s::Input6z parent;
        parent.material=material;
        Require(fe::solid6z::InitializeReference(wedge,parent.reference)==fe::solid6z::Status::Success);
        c.push_back(parent);
      }
    }
    Require(a.size()==908&&b.size()==1309&&c.size()==195);
    for (auto it=unique.rbegin();it!=unique.rend();++it) nodes.push_back({it->first,it->second});
    Require(bool(domain.Initialize({777,nodes.data(),nodes.size()},fe::NodalDomainLimits::Vehicle())));
    Require(bool(model.Initialize(domain,{777,{a.data(),a.size()},{b.data(),b.size()},{c.data(),c.size()}})));
  }
};
} // namespace solid_model_test
