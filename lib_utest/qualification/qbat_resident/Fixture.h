// SPDX-License-Identifier: MIT
#pragma once
#include "../qbat_catalog/Fixture.h"
#include "../qbat_force/Fixture.h"
#include "lib_src/elements/qbat/QbatBatchStartup.h"
#include "lib_src/elements/qbat/QbatBatchAdvance.h"
#include "lib_src/elements/qbat/QbatBatchResultChecks.h"

namespace qbat_resident_test {
namespace fe=tl::fea;
namespace qb=fe::qbat;
namespace batch=qb::batch_detail;
using qbat_force_test::Bytes;
using qbat_force_test::StateValues;
// Bounded host packet construction only; no owner or runtime admission.
struct Source {
  qbat_catalog_test::Fixture fixture;
  fe::ShellBatchBinding binding;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  explicit Source(bool layered=false) {
    // Native TAB1 owns noncentered glass layers. The ordinary synthetic mode
    // uses centered Q/T references to exercise a mixed family without that law.
    if(!layered) {
      for(auto& quad:fixture.geometry.q) quad.reference.placement=fe::ShellReferencePlacement::Centered;
      for(auto& triangle:fixture.triangles) triangle.reference.placement=fe::ShellReferencePlacement::Centered;
    } else {
      for(auto& item:fixture.failures) {
        if(item.source.material_id==2000524) continue;
        item.policy=fe::ShellFailurePolicy::Tab1AnyPoint;
        item.constant={};
        item.tab1.table={{-1,0,1},1};
      }
    }
    EXPECT_EQ(binding.InitializeFormulations(fixture.Geometry()).status,fe::ShellBindingStatus::Success);
    EXPECT_EQ(catalog.InitializeFormulations(binding,fixture.Input()).status,fe::ShellPlasticityBindingStatus::Success);
    EXPECT_EQ(failure.Initialize(catalog,fixture.failures.data(),fixture.failures.size()).status,
        fe::ShellPlasticityBindingStatus::Success);
  }
  fe::ShellFormulationScope Scope() const { return {&binding,&catalog,&failure,nullptr}; }
  qb::BatchConfig Config() const {
    qb::BatchConfig config;
    auto& owner=config.owner;
    owner.owner_id=71;
    owner.node_count=binding.node_count();
    owner.has_rotations=true;
    owner.fixed_dt=0x1p-20;
    owner.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    owner.velocity_phase=fe::NodalVelocityPhase::Collocated;
    config.configuration_id=101;
    config.qualification_id=102;
    config.element_count=binding.qbat_count();
    config.usage=qb::BatchUsage::CoupledForces;
    return config;
  }
};
struct MidlayerSource {
  qbat_binding_test::Fixture geometry;
  fe::ShellBatchBinding binding;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  explicit MidlayerSource(bool triangle=true,double d1=2.5) {
    const fe::ShellFormulationCollectionInput collection{
        {nullptr,triangle?&geometry.t:nullptr,0,triangle?1u:0u,triangle?5u:4u},&geometry.b,1};
    EXPECT_EQ(binding.InitializeFormulations(collection).status,fe::ShellBindingStatus::Success);
    const auto material=qbat_catalog_test::Midlayer();
    const auto section=qbat_catalog_test::MidlayerSection();
    const fe::ShellPlasticityParentInput parents[]{
        {fe::ShellBindingFamily::Qbat,0,geometry.b.source_parent_id,2000524,2000524,2000524},
        {fe::ShellBindingFamily::T3,0,geometry.t.source_parent_id,2000524,2000524,2000524}};
    const fe::ShellBatchPlasticityBindingInput input{nullptr,&material,&section,parents,0,1,1,triangle?2u:1u};
    EXPECT_EQ(catalog.InitializeFormulations(binding,input).status,fe::ShellPlasticityBindingStatus::Success);
    const fe::ShellFailureParentInput failures[]{qbat_catalog_test::Failure(parents[0],d1),qbat_catalog_test::Failure(parents[1],d1)};
    EXPECT_EQ(failure.Initialize(catalog,failures,triangle?2u:1u).status,fe::ShellPlasticityBindingStatus::Success);
  }
  fe::ShellFormulationScope Scope() const { return {&binding,&catalog,&failure,nullptr}; }
};
inline batch::Element Element(const qbat_force_test::Fixture& fixture) {
  batch::Element result;
  result.reference=fixture.reference;
  result.material=fixture.material;
  result.failure=fixture.failure;
  result.source_parent_id=2357655;
  for(unsigned i=0;i<4;++i) result.nodes[i]=i;
  return result;
}
inline qb::ForceTrial Restore(const batch::Element& element,const qb::BatchResult& row) {
  qb::ForceTrial result;
  EXPECT_EQ(qb::PreparePrescribedHistory(element.reference,element.material,element.failure,
      row.history,row.stamp,result.proposed_history),qb::Status::kSuccess);
  result.kinematics=row.kinematics;
  result.diagnostics=row.diagnostics;
  for(unsigned point=0;point<4;++point) {
    result.point[point]=row.point[point];
    result.internal_force_n[point]=row.internal_force_n[point];
    result.internal_couple_nm[point]=row.internal_couple_nm[point];
  }
  return result;
}
} // namespace qbat_resident_test
