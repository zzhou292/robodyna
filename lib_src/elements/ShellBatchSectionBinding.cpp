#include "ShellBatchSectionBinding.h"
#include "ShellBatchPlasticityBindingInternal.h"

namespace tl::fea {
using namespace shell_plasticity_binding_detail;
ShellPlasticityBindingReport ShellBatchPlasticityBinding::InitializeSections(
    const ShellBatchBinding& binding,const ShellBatchPlasticityBindingInput& input,
    const ShellHostBindingLimits& limits) noexcept {
  if(prepared_) return Error(Status::AlreadyInitialized,"Section binding is immutable after preparation");
  // Preserve the explicit small-profile boundary before any borrowed scan.
  if(limits.max_parents>MaxShellHostParents||limits.max_nodes>MaxShellHostNodes)
    return Error(Status::ResourceLimit,"Use explicit section catalog limits for vehicle-sized bindings");
  return InitializeCatalogImpl(binding,input,{limits.max_parents,limits.max_nodes,
    limits.max_parents,limits.max_owned_bytes,MaxVehiclePlasticityCatalogScratchBytes},true);
}
ShellPlasticityBindingReport ShellBatchPlasticityBinding::InitializeSectionCatalog(
    const ShellBatchBinding& binding,const ShellBatchPlasticityBindingInput& input,
    const ShellPlasticityCatalogLimits& limits) noexcept {
  return InitializeCatalogImpl(binding,input,limits,true);
}
bool ShellBatchPlasticityBinding::ElasticParameters(ShellBindingFamily family,std::size_t index,
    material::ShellElasticLaw1PointParameters* output) const noexcept {
  const auto* m=ParentMaterial(family,index);
  if(!output||!m||m->declaration.law!=ShellSectionLaw::LayeredLaw1Nip3) return false;
  const auto& d=m->declaration;
  return material::PrepareShellElasticLaw1Point(d.young_pa,d.poisson_ratio,d.density_kg_m3,*output);
}
bool ShellBatchPlasticityBinding::Law(ShellBindingFamily family,std::size_t index,
    ShellSectionLaw* output) const noexcept {
  const auto* m=ParentMaterial(family,index);
  if(!output||!m) return false;
  const auto* parent=FamilyParent(family,index);
  const auto& section=data_.sections[parent->section_index];
  if(section.formulation==ShellSectionFormulation::OneThicknessPoint) {
    if(family==ShellBindingFamily::Qeph||m->declaration.law!=ShellSectionLaw::LayeredLaw44Nip3) return false;
    *output=family==ShellBindingFamily::Qbat?ShellSectionLaw::Law44QbatFourInPlane:ShellSectionLaw::Law44Nip1;
    return true;
  }
  if(section.formulation!=ShellSectionFormulation::LayeredNip3||
      (m->declaration.law!=ShellSectionLaw::LayeredLaw1Nip3&&
       m->declaration.law!=ShellSectionLaw::LayeredLaw44Nip3)) return false;
  *output=m->declaration.law;
  return true;
}
bool ShellBatchPlasticityBinding::Counts(ShellBindingFamily family,ShellSectionCounts* output) const noexcept {
  if(!prepared_||!output) return false;
  switch(family) {
    case ShellBindingFamily::Qeph: *output=data_.qeph_laws; return true;
    case ShellBindingFamily::T3: *output=data_.t3_laws; return true;
    case ShellBindingFamily::Qbat:
      if(!data_.formulations) return false;
      *output=data_.qbat_laws;
      return true;
    default: return false;
  }
}
} // namespace tl::fea
