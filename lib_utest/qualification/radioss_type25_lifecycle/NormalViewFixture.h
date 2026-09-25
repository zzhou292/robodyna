// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Assertions.h"
#include "NativeOracle.h"
#include "lib_src/collision/radioss_type25/startup/Types.h"
#include <limits>
#include <type_traits>
namespace type25_lifecycle_test::normal_view {
static_assert(std::is_same<n::startup::NormalReference,l::NormalReference>::value);
struct Fields {
  std::vector<n::StoredNormal> faces;
  std::vector<l::NormalReference> references;
  explicit Fields(const Fixture& fixture):references(fixture.normals) {
    for(const auto& main:fixture.mains)for(const auto& slot:main.normal_slot)faces.push_back(slot);
  }
  l::CurrentNormalView View() const {return {faces.data(),faces.size(),references.data(),references.size()};}
  l::Input Bind(const Fixture& fixture) const {
    auto in=fixture.Input();in.current_normals=View();return in;
  }
  // Qualification-only adapter for the independent native oracle and the old
  // static API. Production never copies a Main roster to update normal fields.
  Fixture LegacyControl(const Fixture& fixture) const {
    auto control=fixture;
    if(faces.size()!=4*control.mains.size()||references.size()!=control.normals.size())
      throw std::runtime_error("Incomplete current-normal control fixture");
    for(std::size_t m=0;m<control.mains.size();++m)for(unsigned j=0;j<4;++j)
      control.mains[m].normal_slot[j]=faces[4*m+j];
    control.normals=references;return control;
  }
  void Scale(float value) {
    for(auto& slot:faces){slot.x*=value;slot.y*=value;slot.z*=value;}
    for(auto& ref:references)if(ref.boundary)for(auto& slot:ref.bisector){slot.x*=value;slot.y*=value;slot.z*=value;}
  }
};
inline void PoisonLegacy(Fixture& fixture) {
  const auto poison=std::numeric_limits<float>::quiet_NaN();
  for(auto& main:fixture.mains)for(auto& slot:main.normal_slot)slot={poison,poison,poison};
  for(auto& ref:fixture.normals){ref.boundary=2;for(auto& slot:ref.bisector)slot={poison,poison,poison};}
}
inline Fixture Scenario(unsigned scenario) {
  Fixture f;
  if(scenario==1){f.Retained();f.positions[18]=5;f.velocities[18]=3000;}
  if(scenario==2){f.spatial.push_back({1,1});f.Rebuild();}
  if(scenario==3)f.positions[20]=-.2;
  if(scenario==4){f.Retained();f.positions[20]=2;}
  if(scenario==5)f.TrianglePair();
  if(scenario==6){f.TrianglePair();f.Retained();f.positions[18]=5;f.positions[19]=1;}
  return f;
}
inline l::Report Evaluate(const Fixture& fixture,const Fields& fields,l::HostResult& out,
    l::Limits limits=Fixture::Limits()) {return l::EvaluateNativeLifecycleHost(fields.Bind(fixture),limits,&out);}
inline void SameReport(const l::Report& a,const l::Report& b) {
  EXPECT_EQ(a.status,b.status);EXPECT_EQ(a.stage,b.stage);EXPECT_EQ(a.secondary,b.secondary);EXPECT_EQ(a.occurrence,b.occurrence);
  EXPECT_EQ(a.required_candidates,b.required_candidates);EXPECT_EQ(a.count_complete,b.count_complete);
}
inline void SameFloat(n::StoredNormal a,n::StoredNormal b) {
  const auto bits=[](float x){std::uint32_t v;std::memcpy(&v,&x,sizeof v);return v;};
  EXPECT_EQ(bits(a.x),bits(b.x));EXPECT_EQ(bits(a.y),bits(b.y));EXPECT_EQ(bits(a.z),bits(b.z));
}
} // namespace type25_lifecycle_test::normal_view
