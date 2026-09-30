// SPDX-License-Identifier: AGPL-3.0-or-later
#include "QbatBatchStartup.h"
#include "QbatBatchAdvance.h"
#include "../ShellCatalogCurveView.h"

namespace tl::fea::qbat::batch_detail {
BatchReport BuildElements(const BatchConfig& config,const ShellFormulationScope& scope,
    Storage& storage,BatchDiagnostics& next) noexcept {
  auto& model=storage.model;
  const shell_batch_plasticity_detail::CatalogCurveView curves(*scope.catalog);
  if(curves.count!=model.curve_points) return {BatchStatus::InvalidInput,"QBAT curve pool extent differs"};
  for(std::size_t point=0;point<curves.count;++point) {
    model.curve_x[point]=curves.x[point];
    model.curve_y[point]=curves.y[point];
  }
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
  return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::qbat::batch_detail
