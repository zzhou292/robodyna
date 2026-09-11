// SPDX-License-Identifier: AGPL-3.0-or-later
#include "QbatBatchStartup.h"
#include "QbatBatchAdvance.h"
#include "../ShellBatchJoinedModel.h"
#include "../ShellCatalogCurveView.h"
#include "../../assembly/NodalMassBinding.h"
#include "../../solvers/NodalNativePhysicalCoefficients.h"
#include <cmath>

namespace tl::fea::qbat::batch_detail {
BatchReport ValidateStartup(const BatchConfig& config,const ShellFormulationScope& scope) noexcept {
  const auto checked=ValidateShellFormulationScope(scope);
  if(checked.status!=ShellPlasticityBindingStatus::Success) {
    return {BatchStatus::InvalidInput,checked.message};
  }
  const auto& owner=config.owner;
  if(!owner.owner_id||!owner.has_rotations||owner.epoch||owner.time!=0||owner.velocity_time!=0||
      owner.reactions_valid||!std::isfinite(owner.fixed_dt)||owner.fixed_dt<=0||
      owner.temporal_scheme!=NodalTemporalScheme::StaggeredHalfKickStart||
      owner.velocity_phase!=NodalVelocityPhase::Collocated||!config.configuration_id||!config.qualification_id||
      (config.usage!=BatchUsage::CoupledForces&&config.usage!=BatchUsage::PrescribedFields)||
      owner.node_count!=scope.binding->node_count()||config.element_count!=scope.binding->qbat_count()) {
    return {BatchStatus::InvalidInput,"QBAT requires its complete family and a fresh staggered rotational owner"};
  }
  const bool coupled=config.usage==BatchUsage::CoupledForces;
  if(!native_physical_coefficients::ValidScope(owner.rigid_groups,owner.node_count)||
      (!native_physical_coefficients::Empty(owner.rigid_groups)&&!coupled)||
      (scope.mass&&!coupled)||!shell_startup_detail::ValidStartup(config.startup,coupled)) {
    return {BatchStatus::InvalidInput,"Unsupported QBAT rigid/mass or initial-motion scope"};
  }
  return {BatchStatus::Success,"OK"};
}

BatchReport BuildStartup(const BatchConfig& config,const ShellFormulationScope& scope,
    Storage& storage,BatchDiagnostics& diagnostics) noexcept {
  auto report=ValidateStartup(config,scope);
  if(report.status!=BatchStatus::Success) return report;
  auto& model=storage.model;
  model.config=config;
  shell_batch_detail::ApplyJoinedMass(*scope.binding,model);
  if(scope.mass) {
    for(std::size_t node=0;node<config.owner.node_count;++node) {
      model.mass[node]=scope.mass->nodes()[node].coefficients.mass;
      model.inertia[node]=scope.mass->nodes()[node].coefficients.isotropic_inertia;
    }
  }
  const shell_batch_plasticity_detail::CatalogCurveView curves(*scope.catalog);
  if(curves.count!=model.curve_points) return {BatchStatus::InvalidInput,"QBAT curve pool extent differs"};
  for(std::size_t point=0;point<curves.count;++point) {
    model.curve_x[point]=curves.x[point];
    model.curve_y[point]=curves.y[point];
  }
  BatchDiagnostics next;
  next.owner_id=config.owner.owner_id;
  next.configuration_id=config.configuration_id;
  next.qualification_id=config.qualification_id;
  next.phase=BatchPhase::Accepted;
  next.usage=config.usage;
  next.element_count=config.element_count;
  next.active_count=config.element_count;
  for(std::size_t parent=0;parent<config.element_count;++parent) {
    auto& element=model.element[parent];
    element.reference=scope.binding->qbat_reference(parent);
    element.source_parent_id=scope.binding->qbat_source_id(parent);
    for(unsigned local=0;local<4;++local) element.nodes[local]=scope.binding->qbat_nodes(parent)[local];
    const auto* failure=scope.failure->parent(ShellBindingFamily::Qbat,parent);
    ShellSectionLaw law{};
    if(!scope.catalog->Law(ShellBindingFamily::Qbat,parent,&law)||law!=ShellSectionLaw::Law44QbatFourInPlane||
        !scope.catalog->Parameters(ShellBindingFamily::Qbat,parent,&element.material)||!failure||
        failure->policy!=ShellFailurePolicy::ConstantAllPoints) {
      return {BatchStatus::InvalidInput,"QBAT requires its resolved four-point constant-failure section",
          static_cast<std::uint32_t>(parent)};
    }
    element.failure=failure->constant;
    std::size_t offset=0;
    if(!curves.Offset(element.material,offset)) return {BatchStatus::InvalidInput,"QBAT catalog curve view differs"};
    const auto status=InitializeResult(element,storage.slab[0].element[parent]);
    if(status!=Status::kSuccess) {
      return {BatchStatus::ElementFailure,"QBAT native startup history rejected",
          static_cast<std::uint32_t>(parent),UINT32_MAX,status};
    }
    if(offset!=NoShellBindingNode) {
      element.material.curve.plastic_strain=model.curve_x+offset;
      element.material.curve.yield_stress_pa=model.curve_y+offset;
    }
    const double dt=element.reference.coefficients().unscaled_element_dt_s;
    if(!parent||dt<next.minimum_native_dt) next.minimum_native_dt=dt;
  }
  double declared_kinetic=0;
  for(std::size_t node=0;node<config.owner.node_count;++node) {
    if(!detail::Positive(model.mass[node])||!detail::Positive(model.inertia[node])||
        !detail::Positive(model.physical[node])||!detail::Positive(model.added[node])||
        !shell_startup_detail::AddInitialTranslationKinetic(model.mass[node],
            config.startup.uniform_velocity,declared_kinetic)) {
      return {BatchStatus::InvalidMass,"QBAT complete nodal coefficients or initial kinetic domain are invalid",
          UINT32_MAX,static_cast<std::uint32_t>(node)};
    }
  }
  next.valid=true;
  diagnostics=next;
  return {BatchStatus::Success,"OK"};
}

bool RebaseMaterials(Storage& host,const Storage& device,const ShellBatchPlasticityBinding& catalog) noexcept {
  const shell_batch_plasticity_detail::CatalogCurveView curves(catalog);
  for(std::size_t parent=0;parent<host.model.config.element_count;++parent) {
    Material parameters;
    std::size_t offset=0;
    if(!catalog.Parameters(ShellBindingFamily::Qbat,parent,&parameters)||!curves.Offset(parameters,offset)) return false;
    auto& material=host.model.element[parent].material;
    if(offset!=NoShellBindingNode) {
      material.curve.plastic_strain=device.model.curve_x+offset;
      material.curve.yield_stress_pa=device.model.curve_y+offset;
    }
  }
  return true;
}
} // namespace tl::fea::qbat::batch_detail
