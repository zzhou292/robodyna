// SPDX-License-Identifier: MIT
#pragma once
#include "../qbat_binding/Fixture.h"
#include "lib_src/elements/ShellFormulationScope.h"
#include "lib_src/materials/TabulatedShellPlasticity.h"
#include <memory>

namespace qbat_catalog_test {
namespace fe=tl::fea;
namespace mat=tl::material;
using Catalog=fe::ShellBatchPlasticityBinding;
using Status=fe::ShellPlasticityBindingStatus;
using Family=fe::ShellBindingFamily;
using qbat_binding_test::Bytes;
using qbat_binding_test::Bits;

inline fe::ShellPlasticityMaterialInput Midlayer() {
  fe::ShellPlasticityMaterialInput m;
  m.material_id=2000524;
  m.young_pa=250e6;
  m.poisson_ratio=.35;
  m.density_kg_m3=1000;
  m.hardening=mat::ShellPlasticityHardeningKind::LinearLaw44;
  m.linear={10e6,1e6};
  m.rate={true,0,1,10000,mat::ShellPlasticityRatePolicy::FilteredZeroC};
  return m;
}
inline fe::ShellPlasticitySectionInput MidlayerSection() {
  return {2000524,.0005,1,fe::ShellSectionFormulation::OneThicknessPoint};
}
inline fe::ShellFailureParentInput Failure(fe::ShellPlasticityParentInput p,double d1=2.5) {
  fe::ShellFailureParentInput f;
  f.source=p;
  f.policy=fe::ShellFailurePolicy::ConstantAllPoints;
  f.constant.failure_strain=d1;
  return f;
}

// Two synthetic glass layers share every Q4 and T3 node with a midlayer.
// The two top/bottom layers have distinct EIDs/PIDs; each PID retains one SID
// and MID across its quad/triangle. This is a declaration test, not a runtime.
struct Fixture {
  qbat_binding_test::Fixture geometry;
  std::array<fe::ShellT3BindingInput,3> triangles;
  double x[3]{0,.02,.1},y[3]{10e6,15e6,20e6};
  fe::ShellPlasticityCurveInput curve;
  std::array<fe::ShellPlasticityMaterialInput,3> materials;
  std::array<fe::ShellPlasticitySectionInput,3> sections;
  std::array<fe::ShellPlasticityParentInput,6> parents;
  std::array<fe::ShellFailureParentInput,6> failures;
  Fixture() {
    curve={101,{x,y,3}};
    for(unsigned i=0;i<2;++i) {
      triangles[i]=geometry.t;
      triangles[i].source_parent_id=200+i;
      auto& r=triangles[i].reference;
      const auto& q=geometry.q[i].reference;
      r.density=q.density;
      r.young_modulus=q.young_modulus;
      r.poisson_ratio=q.poisson_ratio;
      r.thickness=q.thickness;
      r.placement=q.placement;
      materials[i]={1000+i,101,q.young_modulus,q.poisson_ratio,q.density};
      sections[i]={1000+i,q.thickness,3};
    }
    triangles[2]=geometry.t;
    materials[2]=Midlayer();
    sections[2]=MidlayerSection();
    // Deliberately interleave family rows, retaining this exact source order.
    parents={{{Family::Qbat,0,geometry.b.source_parent_id,2000524,2000524,2000524},
        {Family::T3,1,201,1001,1001,1001},
        {Family::Qeph,0,100,1000,1000,1000},
        {Family::T3,2,geometry.t.source_parent_id,2000524,2000524,2000524},
        {Family::Qeph,1,101,1001,1001,1001},
        {Family::T3,0,200,1000,1000,1000}}};
    for(unsigned i=0;i<parents.size();++i) failures[i]=Failure(parents[i]);
  }
  fe::ShellFormulationCollectionInput Geometry() const {
    return {{geometry.q.data(),triangles.data(),2,3,5},&geometry.b,1};
  }
  fe::ShellBatchPlasticityBindingInput Input() const {
    return {&curve,materials.data(),sections.data(),parents.data(),1,3,3,6};
  }
};

inline void CheckMidlayer(const Catalog& catalog,Family family,std::size_t index,
                          fe::ShellSectionLaw expected) {
  fe::ShellSectionLaw law{};
  ASSERT_TRUE(catalog.Law(family,index,&law));
  EXPECT_EQ(law,expected);
  fe::sections::PointParameters p;
  ASSERT_TRUE(catalog.Parameters(family,index,&p));
  EXPECT_EQ(p.curve.count,0u);
  EXPECT_EQ(p.curve.plastic_strain,nullptr);
  EXPECT_EQ(p.curve.yield_stress_pa,nullptr);
  EXPECT_EQ(p.hardening,mat::ShellPlasticityHardeningKind::LinearLaw44);
  EXPECT_EQ(p.rate.policy,mat::ShellPlasticityRatePolicy::FilteredZeroC);
  EXPECT_EQ(Bits(p.a11),Bits(250e6/(1-.35*.35)));
}
} // namespace qbat_catalog_test
