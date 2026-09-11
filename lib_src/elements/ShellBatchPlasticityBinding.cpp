#include "ShellBatchPlasticityBindingInternal.h"

namespace tl::fea {
using shell_plasticity_binding_detail::Same;
bool ShellBatchPlasticityBinding::Matches(const ShellBatchBinding& binding) const noexcept {
  return prepared_&&binding.prepared()&&data_.inventory==binding.inventory();
}
const ShellPlasticityParentInput* ShellBatchPlasticityBinding::parent(std::size_t index) const noexcept {
  return prepared_&&index<data_.parent_count?&data_.parents[index].declaration:nullptr;
}
const ShellBatchPlasticityBinding::Parent* ShellBatchPlasticityBinding::FamilyParent(
    ShellBindingFamily family,std::size_t index) const noexcept {
  if(!prepared_) return nullptr;
  switch(family) {
    case ShellBindingFamily::Qeph:
      return index<data_.qeph_count?&data_.parents[data_.qeph_parent[index]]:nullptr;
    case ShellBindingFamily::T3:
      return index<data_.t3_count?&data_.parents[data_.t3_parent[index]]:nullptr;
    case ShellBindingFamily::Qbat:
      return data_.formulations&&index<data_.qbat_count?&data_.parents[data_.qbat_parent[index]]:nullptr;
    default:
      return nullptr;
  }
}
const ShellBatchPlasticityBinding::Material* ShellBatchPlasticityBinding::ParentMaterial(
    ShellBindingFamily family,std::size_t index) const noexcept {
  const auto* parent=FamilyParent(family,index);
  return parent?&data_.materials[parent->material_index]:nullptr;
}
bool ShellBatchPlasticityBinding::Parameters(ShellBindingFamily family,std::size_t index,
    sections::PointParameters* output) const noexcept {
  const auto* material=ParentMaterial(family,index);
  if(!output||!material||material->declaration.law!=ShellSectionLaw::LayeredLaw44Nip3) return false;
  auto result=material->coefficients;
  if(result.hardening==material::ShellPlasticityHardeningKind::Tabulated) {
    const auto& curve=data_.curves[material->curve_index];
    result.curve={data_.curve_x.data()+curve.offset,data_.curve_y.data()+curve.offset,
      static_cast<std::uint32_t>(curve.count)};
  }
  *output=result; return true;
}
bool ShellBatchPlasticityBinding::SameScope(const ShellBatchPlasticityBinding& other) const noexcept {
  const auto& a=data_; const auto& b=other.data_;
  if(!prepared_||!other.prepared_||a.inventory!=b.inventory||a.curve_count!=b.curve_count||
      a.material_count!=b.material_count||a.section_count!=b.section_count||a.parent_count!=b.parent_count||
      a.point_count!=b.point_count||a.qeph_count!=b.qeph_count||a.t3_count!=b.t3_count||
      a.qbat_count!=b.qbat_count||a.formulations!=b.formulations||a.heterogeneous!=b.heterogeneous) return false;
  for(std::size_t i=0;i<a.curve_count;++i)
    if(a.curves[i].id!=b.curves[i].id||a.curves[i].offset!=b.curves[i].offset||a.curves[i].count!=b.curves[i].count) return false;
  for(std::size_t i=0;i<a.point_count;++i)
    if(!Same(a.curve_x[i],b.curve_x[i])||!Same(a.curve_y[i],b.curve_y[i])) return false;
  for(std::size_t i=0;i<a.material_count;++i) {
    const auto& x=a.materials[i]; const auto& y=b.materials[i];
    const auto& m=x.declaration; const auto& n=y.declaration;
    if(x.curve_index!=y.curve_index||m.material_id!=n.material_id||m.curve_id!=n.curve_id||
        !Same(m.young_pa,n.young_pa)||!Same(m.poisson_ratio,n.poisson_ratio)||
        !Same(m.density_kg_m3,n.density_kg_m3)||!Same(m.rate,n.rate)||m.hardening!=n.hardening||m.law!=n.law||
        m.continuation!=n.continuation||
        !Same(m.linear.initial_yield_pa,n.linear.initial_yield_pa)||
        !Same(m.linear.tangent_modulus_pa,n.linear.tangent_modulus_pa)) return false;
  }
  for(std::size_t i=0;i<a.section_count;++i) {
    const auto& x=a.sections[i]; const auto& y=b.sections[i];
    if(x.section_id!=y.section_id||!Same(x.thickness_m,y.thickness_m)||
        x.through_thickness_points!=y.through_thickness_points||x.formulation!=y.formulation) return false;
  }
  for(std::size_t i=0;i<a.parent_count;++i) {
    const auto& x=a.parents[i]; const auto& y=b.parents[i];
    const auto& p=x.declaration; const auto& q=y.declaration;
    if(x.material_index!=y.material_index||x.section_index!=y.section_index||p.family!=q.family||
        p.family_index!=q.family_index||p.source_parent_id!=q.source_parent_id||p.source_part_id!=q.source_part_id||
        p.material_id!=q.material_id||p.section_id!=q.section_id) return false;
  }
  // Family lookup arrays and pointer-free coefficients are derived solely from
  // the complete checked declarations above; no reduced hash grants equality.
  return true;
}
} // namespace tl::fea
