// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <gtest/gtest.h>

namespace solid_model_test {
TEST(SolidModel, AllFamiliesShareCanonicalMidAndExactGlobalMaps) {
  Fixture fixture;
  const auto domain=fixture.Domain();
  s::Model model;
  ASSERT_TRUE(model.Initialize(domain,fixture.Input()));
  ASSERT_EQ(model.solid18().size(),1u);ASSERT_EQ(model.solid24().size(),1u);ASSERT_EQ(model.solid6z().size(),1u);
  EXPECT_EQ(model.materials36().size(),1u);EXPECT_EQ(model.materials42().size(),1u);
  EXPECT_EQ(model.solid24()[0].material_index,model.solid6z()[0].material_index);
  EXPECT_EQ(model.source_instance_id(),777u);
  EXPECT_TRUE(model.domain()->SharesStorage(domain));
  for(const auto& row:model.contributions()->parents()) {
    for(unsigned n=0;n<row.node_count;++n)
      EXPECT_EQ(model.domain()->nodes()[row.domain_node[n]].source_id,row.source_node_id[n]);
  }
  for(unsigned n=0;n<8;++n)EXPECT_EQ(model.solid24()[0].domain_nodes[n],domain.Find(n+1));
  EXPECT_EQ(model.solid6z()[0].domain_nodes[6],SIZE_MAX);
  EXPECT_EQ(model.solid6z()[0].domain_nodes[7],SIZE_MAX);
  s::Model independently_owned;
  Fixture other;
  ASSERT_TRUE(independently_owned.Initialize(other.Domain(),other.Input()));
  EXPECT_TRUE(model.Matches(independently_owned));
  EXPECT_FALSE(model.SharesStorage(independently_owned));
}
TEST(SolidModel, CurvesAndSourceBackingOutliveInputsAndDeduplicateRepeatedMid) {
  std::unique_ptr<s::Model> owner;
  const double* borrowed=nullptr;
  {
    Fixture f;
    auto repeated=f.input18[0];
    auto input=repeated.reference.input();input.source_element_id=104;
    ASSERT_EQ(fe::solid18::InitializeReference(input,repeated.reference),fe::solid18::Status::Success);
    f.input18.push_back(repeated);
    owner=std::make_unique<s::Model>();
    ASSERT_TRUE(owner->Initialize(f.Domain(),f.Input()));
    borrowed=f.strain;
    EXPECT_NE(owner->materials36()[0].value.curve.plastic_strain,borrowed);
    f.strain[1]=.19;f.stress[2]=9e6;
  }
  ASSERT_EQ(owner->materials36().size(),1u);
  EXPECT_EQ(owner->solid18()[0].material_index,owner->solid18()[1].material_index);
  const s::Model retained=*owner;owner.reset();
  EXPECT_EQ(retained.materials36()[0].value.curve.plastic_strain[1],.2);
  EXPECT_EQ(retained.materials36()[0].value.curve.yield_stress_pa[2],3e6);
  EXPECT_EQ(retained.domain()->node_count(),9u);
  s::Model copy=retained;EXPECT_TRUE(copy.SharesStorage(retained));
}
TEST(SolidModel, LateMaterialAndDuplicateElementRejectThenRetry) {
  Fixture f;const auto domain=f.Domain();const auto good=f.input6z[0];
  ASSERT_EQ(tl::material::law42::Prepare(25e6,.463,1980,1e26,f.input6z[0].material),tl::material::law42::Status::Ok);
  s::Model model;
  auto report=model.Initialize(domain,f.Input());
  EXPECT_EQ(report.status,s::ModelStatus::MaterialMismatch);
  EXPECT_EQ(report.family,s::Family::Solid6z);EXPECT_EQ(report.parent,0u);EXPECT_FALSE(model.prepared());
  f.input6z[0]=good;
  auto duplicate=good.reference.input();duplicate.source_element_id=101;
  ASSERT_EQ(fe::solid6z::InitializeReference(duplicate,f.input6z[0].reference),fe::solid6z::Status::Success);
  report=model.Initialize(domain,f.Input());
  EXPECT_EQ(report.status,s::ModelStatus::DuplicateIdentity);EXPECT_FALSE(model.prepared());
  f.input6z[0]=good;
  ASSERT_TRUE(model.Initialize(domain,f.Input()));
  const auto before=model.owned_payload_bytes();
  EXPECT_EQ(model.Initialize(domain,f.Input()).status,s::ModelStatus::AlreadyInitialized);
  EXPECT_EQ(model.owned_payload_bytes(),before);
}
TEST(SolidModel, ExactCapAndCountPreflightPrecedeBorrowedReads) {
  Fixture f;const auto domain=f.Domain();s::Model baseline;
  ASSERT_TRUE(baseline.Initialize(domain,f.Input()));
  s::ModelLimits limits;limits.max_host_bytes=baseline.startup_payload_bytes();
  s::Model exact;ASSERT_TRUE(exact.Initialize(domain,f.Input(),limits));
  --limits.max_host_bytes;s::Model short_cap;
  EXPECT_EQ(short_cap.Initialize(domain,f.Input(),limits).status,s::ModelStatus::ResourceLimit);
  auto input=f.Input();
  input.solid18={reinterpret_cast<const s::Input18*>(1),1};
  limits.max_host_bytes=1;s::Model unread;
  EXPECT_EQ(unread.Initialize(domain,input,limits).status,s::ModelStatus::ResourceLimit);
  input.solid18={reinterpret_cast<const s::Input18*>(1),SIZE_MAX};
  EXPECT_EQ(unread.Initialize(domain,input).status,s::ModelStatus::ResourceLimit);
}
TEST(SolidModel, MechanicsIdentityIncludesProfileDespiteEqualCoefficients) {
  Fixture a,b;
  b.input6z[0].profile.damping_coefficient=.2;
  s::Model first,second;
  ASSERT_TRUE(first.Initialize(a.Domain(),a.Input()));ASSERT_TRUE(second.Initialize(b.Domain(),b.Input()));
  EXPECT_TRUE(first.contributions()->Matches(*second.contributions()));EXPECT_FALSE(first.Matches(second));
  Fixture units;
  auto millimetre=units.input24[0].reference.input();
  millimetre.profile.working_length=fe::solid24::WorkingLengthUnit::Millimetre;
  ASSERT_EQ(fe::solid24::InitializeReference(millimetre,units.input24[0].reference),fe::solid24::Status::Success);
  s::Model different_units;
  ASSERT_TRUE(different_units.Initialize(units.Domain(),units.Input()));
  EXPECT_TRUE(first.contributions()->Matches(*different_units.contributions()));
  EXPECT_FALSE(first.Matches(different_units));
  auto local_only=a.input24[0].reference.input();
  local_only.profile.reference_strain=fe::solid24::ReferenceStrain::LocalGeometryOnly;
  ASSERT_EQ(fe::solid24::InitializeReference(local_only,a.input24[0].reference),fe::solid24::Status::Success);
  s::Model rejected;EXPECT_EQ(rejected.Initialize(a.Domain(),a.Input()).status,s::ModelStatus::InvalidInput);
}
TEST(SolidModel, CurveTailAndSignedZeroAndLateDomainMismatchRemainExact) {
  Fixture a,b;const auto domain=a.Domain();
  b.strain[0]=-0.0;
  s::Model first,second;
  ASSERT_TRUE(first.Initialize(domain,a.Input()));ASSERT_TRUE(second.Initialize(b.Domain(),b.Input()));
  EXPECT_FALSE(first.Matches(second));
  b.stress[2]=std::numeric_limits<double>::quiet_NaN();
  s::Model rejected;
  EXPECT_EQ(rejected.Initialize(b.Domain(),b.Input()).status,s::ModelStatus::InvalidInput);
  EXPECT_FALSE(rejected.prepared());
  auto changed=a.input6z[0].reference.input();changed.position_m[5].z+=1e-8;
  ASSERT_EQ(fe::solid6z::InitializeReference(changed,a.input6z[0].reference),fe::solid6z::Status::Success);
  EXPECT_EQ(rejected.Initialize(domain,a.Input()).status,s::ModelStatus::SourceMismatch);
  EXPECT_FALSE(rejected.prepared());
}
TEST(SolidModel, ExplicitEmptyFamiliesAndMaterialPoolCaps) {
  for(unsigned mask=1;mask<8;++mask) {
    Fixture f;
    if(!(mask&1))f.input18.clear();
    if(!(mask&2))f.input24.clear();
    if(!(mask&4))f.input6z.clear();
    s::Model model;
    ASSERT_TRUE(model.Initialize(f.Domain(),f.Input()))<<mask;
    EXPECT_EQ(model.solid18().size(),bool(mask&1));
    EXPECT_EQ(model.solid24().size(),bool(mask&2));
    EXPECT_EQ(model.solid6z().size(),bool(mask&4));
    EXPECT_EQ(model.materials36().size(),bool(mask&1));
    EXPECT_EQ(model.materials42().size(),bool(mask&6));
  }
  Fixture f;
  s::ModelLimits limits;limits.max_curve_points=2;
  s::Model rejected;
  EXPECT_EQ(rejected.Initialize(f.Domain(),f.Input(),limits).status,s::ModelStatus::ResourceLimit);
  limits=s::ModelLimits{};limits.max_materials=1;
  EXPECT_EQ(rejected.Initialize(f.Domain(),f.Input(),limits).status,s::ModelStatus::ResourceLimit);
  auto wrong=f.Input();wrong.source_instance_id++;
  EXPECT_EQ(rejected.Initialize(f.Domain(),wrong).status,s::ModelStatus::InvalidInput);
  EXPECT_FALSE(rejected.prepared());
}
} // namespace solid_model_test
