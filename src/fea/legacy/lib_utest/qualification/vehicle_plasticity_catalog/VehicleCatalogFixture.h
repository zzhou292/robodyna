#pragma once
#include "../vehicle_shell_host/VehicleShellFixture.h"
#include "lib_src/elements/ShellBatchPlasticityBinding.h"
#include "../host_shell_collection/AllocationFailure.h"
#include <algorithm>
#include <memory>

namespace vehicle_catalog_test {
namespace fe=tl::fea;
using Catalog=fe::ShellBatchPlasticityBinding;
using Status=fe::ShellPlasticityBindingStatus;
using vehicle_shell_test::Bytes;
using Kind=tl::material::ShellPlasticityHardeningKind;
struct Fixture {
  vehicle_shell_test::Fixture geometry;
  std::array<double,2> x{0.,10.},y{20000.,30000.};
  std::array<fe::ShellPlasticityCurveInput,1> curves;
  std::vector<fe::ShellPlasticityMaterialInput> materials;
  std::vector<fe::ShellPlasticitySectionInput> sections;
  std::vector<fe::ShellPlasticityParentInput> parents;
  Fixture(std::size_t q=2049,std::size_t t=1049,std::size_t nodes=4097,std::size_t definitions=130)
    :geometry(q,t,nodes),materials(definitions),sections(definitions) {
    curves[0]={123,{x.data(),y.data(),2}};
    const auto qdefs=(definitions+1)/2;
    for(std::size_t i=0;i<definitions;++i) {
      const bool isq=i<qdefs;
      auto& m=materials[i];m.material_id=(std::uint64_t{1}<<59)+definitions-i;
      m.young_pa=2e6;m.poisson_ratio=.3;m.density_kg_m3=isq?1024.:7890.;
      m.rate={true,8000.,8.,10000.};
      if(i%2) {m.hardening=Kind::LinearLaw44;m.linear={20000.+double(i),20000.};}
      else m.curve_id=123;
      sections[i]={(std::uint64_t{1}<<61)+definitions-i,isq?1./32:1./128,3};
    }
    parents.reserve(q+t);
    for(std::size_t i=0;i<q;++i) Add(true,i,i%qdefs);
    for(std::size_t i=0;i<t;++i) Add(false,i,qdefs+i%(definitions-qdefs));
    // Original declared order differs from both family order and sorted IDs.
    std::reverse(parents.begin(),parents.end());
  }
  void Add(bool q,std::size_t family,std::size_t definition) {
    parents.push_back({q?fe::ShellBindingFamily::Qeph:fe::ShellBindingFamily::T3,family,
      q?geometry.q[family].source_parent_id:geometry.t[family].source_parent_id,
      (std::uint64_t{1}<<62)+materials.size()-definition,
      materials[definition].material_id,sections[definition].section_id});
  }
  fe::ShellBatchPlasticityBindingInput input() const {
    return {curves.data(),materials.data(),sections.data(),parents.data(),
      curves.size(),materials.size(),sections.size(),parents.size()};
  }
  auto limits() const {return fe::ShellPlasticityCatalogLimits::Vehicle();}
};
inline void PrepareGeometry(Fixture& f,fe::ShellBatchBinding& b) {
  ASSERT_EQ(b.Initialize(f.geometry.input(),fe::ShellHostBindingLimits::Vehicle()).status,
    fe::ShellBindingStatus::Success);
}
inline void CheckParent(const Fixture& f,const Catalog& catalog,std::size_t i) {
  const auto& source=f.parents[i];const auto* p=catalog.parent(i);ASSERT_NE(p,nullptr);
  EXPECT_EQ(p->family,source.family);EXPECT_EQ(p->family_index,source.family_index);
  EXPECT_EQ(p->source_parent_id,source.source_parent_id);EXPECT_EQ(p->source_part_id,source.source_part_id);
  EXPECT_EQ(p->material_id,source.material_id);EXPECT_EQ(p->section_id,source.section_id);
  const auto di=(std::uint64_t{1}<<59)+f.materials.size()-source.material_id;
  const auto& declaration=f.materials[di];fe::sections::PointParameters value;
  ASSERT_TRUE(catalog.Parameters(p->family,p->family_index,&value));
  EXPECT_EQ(value.hardening,declaration.hardening);
  EXPECT_DOUBLE_EQ(value.young_pa,declaration.young_pa);
  EXPECT_DOUBLE_EQ(value.density_kg_m3,declaration.density_kg_m3);
  if(declaration.hardening==Kind::LinearLaw44) {
    EXPECT_EQ(value.curve.plastic_strain,nullptr);EXPECT_EQ(value.curve.yield_stress_pa,nullptr);
    EXPECT_EQ(value.curve.count,0u);EXPECT_DOUBLE_EQ(value.linear.initial_yield_pa,declaration.linear.initial_yield_pa);
  } else {
    ASSERT_NE(value.curve.yield_stress_pa,nullptr);EXPECT_EQ(value.curve.count,2u);
    EXPECT_DOUBLE_EQ(value.curve.yield_stress_pa[0],f.y[0]);
  }
}
} // namespace vehicle_catalog_test
