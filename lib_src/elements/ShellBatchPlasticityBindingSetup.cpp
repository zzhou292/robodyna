#include "ShellBatchPlasticityBindingInternal.h"
#include <new>

namespace tl::fea {
using namespace shell_plasticity_binding_detail;

ShellPlasticityBindingReport ShellBatchPlasticityBinding::Initialize(
    const ShellBatchBinding& binding,const ShellBatchPlasticityBindingInput& input) noexcept {
  return Initialize(binding,input,ShellHostBindingLimits{});
}
std::size_t ShellBatchPlasticityBinding::host_bytes() const noexcept {
  return sizeof(*this)+data_.inventory.backing_bytes()+data_.curves.backing_bytes()+
    data_.materials.backing_bytes()+data_.sections.backing_bytes()+data_.parents.backing_bytes()+
    data_.qeph_parent.backing_bytes()+data_.t3_parent.backing_bytes();
}
ShellPlasticityBindingReport ShellBatchPlasticityBinding::Initialize(
    const ShellBatchBinding& binding,const ShellBatchPlasticityBindingInput& input,
    const ShellHostBindingLimits& limits) noexcept {
  if(prepared_) return Error(Status::AlreadyInitialized,"Plasticity binding is immutable after preparation");
  if(!binding.prepared()||!input.curves||!input.materials||!input.sections||!input.parents||
     !input.curve_count||!input.material_count||!input.section_count||!input.parent_count)
    return Error(Status::InvalidInput,"Complete prepared binding and explicit nonempty catalog ranges are required");
  for(auto count:{input.curve_count,input.material_count,input.section_count,input.parent_count})
    if(count>MaxShellHostParents||count>limits.max_parents)
      return Error(Status::ResourceLimit,"Plasticity catalog exceeds host parent admission");
  if(limits.max_parents>MaxShellHostParents||limits.max_nodes>MaxShellHostNodes||binding.node_count()>limits.max_nodes)
    return Error(Status::ResourceLimit,"Plasticity catalog exceeds host node admission");
  if(input.parent_count!=binding.qeph_count()+binding.t3_count())
    return Error(Status::InvalidParent,"Plasticity mapping must cover every native collection parent");
  const auto bytes=sizeof(*this)+binding.inventory().backing_bytes()+
    decltype(data_.curves)::ExtraBytes(input.curve_count)+
    decltype(data_.materials)::ExtraBytes(input.material_count)+
    decltype(data_.sections)::ExtraBytes(input.section_count)+
    decltype(data_.parents)::ExtraBytes(input.parent_count)+
    decltype(data_.qeph_parent)::ExtraBytes(binding.qeph_count())+
    decltype(data_.t3_parent)::ExtraBytes(binding.t3_count());
  if(bytes>limits.max_owned_bytes)
    return Error(Status::ResourceLimit,"Owned host plasticity catalog exceeds byte admission");
  // All range counts and the TOTAL pool size are checked before any curve copy.
  std::size_t points=0;
  for(std::size_t i=0;i<input.curve_count;++i) {
    const auto& c=input.curves[i];
    if(!c.curve_id||!c.curve.plastic_strain||!c.curve.yield_stress_pa||c.curve.count<2)
      return Error(Status::InvalidCurve,"Curve requires a nonzero ID and at least two supplied points",i);
    if(c.curve.count>MaxShellPlasticityCurvePoints-points)
      return Error(Status::ResourceLimit,"Complete plasticity curve pool exceeds 1024 points",i);
    points+=c.curve.count;
  }
  try { return Build(binding,input); }
  catch(const std::bad_alloc&) { return Error(Status::ResourceLimit,"Plasticity catalog startup allocation failed"); }
}
ShellPlasticityBindingReport ShellBatchPlasticityBinding::Build(
    const ShellBatchBinding& binding,const ShellBatchPlasticityBindingInput& input) {
  Data staged;
  staged.curves.Resize(input.curve_count); staged.materials.Resize(input.material_count);
  staged.sections.Resize(input.section_count); staged.parents.Resize(input.parent_count);
  staged.qeph_parent.Resize(binding.qeph_count()); staged.t3_parent.Resize(binding.t3_count());
  staged.inventory=binding.inventory(); staged.qeph_count=binding.qeph_count(); staged.t3_count=binding.t3_count();
  auto report=CopyCurves(input,staged);
  if(report.status==Status::Success) report=PrepareMaterials(input,staged);
  if(report.status==Status::Success) report=CopySections(input,staged);
  if(report.status==Status::Success) report=BindParents(binding,input,staged);
  if(report.status!=Status::Success) return report;
  data_=staged; prepared_=true; return {};
}

ShellPlasticityBindingReport ShellBatchPlasticityBinding::CopyCurves(
    const ShellBatchPlasticityBindingInput& input,Data& out) noexcept {
  for(std::size_t i=0;i<input.curve_count;++i) {
    const auto& c=input.curves[i];
    if(Find(out.curves,i,c.curve_id,[](const auto& x){return x.id;})!=NoShellBindingNode)
      return Error(Status::InvalidCurve,"Duplicate curve ID",i);
    sections::PointParameters checked;
    if(material::PrepareTabulatedShellPlasticity(1.,.3,1.,c.curve,checked)!=sections::PointStatus::Ok)
      return Error(Status::InvalidCurve,"Invalid tabulated shell curve",i);
    out.curves[i]={c.curve_id,out.point_count,c.curve.count};
    for(std::size_t j=0;j<c.curve.count;++j) {
      out.curve_x[out.point_count+j]=c.curve.plastic_strain[j];
      out.curve_y[out.point_count+j]=c.curve.yield_stress_pa[j];
    }
    out.point_count+=c.curve.count;
  }
  out.curve_count=input.curve_count; return {};
}

ShellPlasticityBindingReport ShellBatchPlasticityBinding::PrepareMaterials(
    const ShellBatchPlasticityBindingInput& input,Data& out) noexcept {
  std::array<bool,MaxShellHostParents> used{};
  for(std::size_t i=0;i<input.material_count;++i) {
    const auto& m=input.materials[i];
    if(!m.material_id||!m.curve_id||Find(out.materials,i,m.material_id,
        [](const auto& x){return x.declaration.material_id;})!=NoShellBindingNode)
      return Error(Status::InvalidMaterial,"Missing or duplicate material ID",i);
    const auto ci=Find(out.curves,out.curve_count,m.curve_id,[](const auto& x){return x.id;});
    if(ci==NoShellBindingNode) return Error(Status::InvalidMaterial,"Material references an absent curve",i);
    const auto& c=out.curves[ci];
    const material::TabulatedShellPlasticityCurve curve{out.curve_x.data()+c.offset,out.curve_y.data()+c.offset,
      static_cast<std::uint32_t>(c.count)};
    auto& target=out.materials[i];
    if(material::PrepareTabulatedShellPlasticity(m.young_pa,m.poisson_ratio,m.density_kg_m3,
        curve,m.rate,target.coefficients)!=sections::PointStatus::Ok)
      return Error(Status::InvalidMaterial,"Material coefficients or rate declaration are invalid",i);
    target.coefficients.curve={}; // Never retain a pointer into staged or another catalog's storage.
    target.declaration=m; target.curve_index=ci; used[ci]=true;
  }
  for(std::size_t i=0;i<out.curve_count;++i)
    if(!used[i]) return Error(Status::UnreferencedDeclaration,"Unreferenced curve declaration",i);
  out.material_count=input.material_count; return {};
}

ShellPlasticityBindingReport ShellBatchPlasticityBinding::CopySections(
    const ShellBatchPlasticityBindingInput& input,Data& out) noexcept {
  for(std::size_t i=0;i<input.section_count;++i) {
    const auto& s=input.sections[i];
    if(!s.section_id||!tl::math::Finite(s.thickness_m)||s.thickness_m<=0||s.through_thickness_points!=3||
        Find(out.sections,i,s.section_id,[](const auto& x){return x.section_id;})!=NoShellBindingNode)
      return Error(Status::InvalidSection,"Section requires unique nonzero ID, positive thickness and NIP3",i);
    out.sections[i]=s;
  }
  out.section_count=input.section_count; return {};
}

ShellPlasticityBindingReport ShellBatchPlasticityBinding::BindParents(const ShellBatchBinding& binding,
    const ShellBatchPlasticityBindingInput& input,Data& out) noexcept {
  std::array<bool,MaxShellHostParents> qseen{},tseen{},materials{},sections{};
  for(std::size_t i=0;i<input.parent_count;++i) {
    const auto& p=input.parents[i];
    if(!p.source_parent_id||!p.source_part_id||!p.material_id||!p.section_id||
       (p.family!=ShellBindingFamily::Qeph&&p.family!=ShellBindingFamily::T3))
      return Error(Status::InvalidParent,"Explicit source parent/part/material/section IDs and family are required",i,p.family);
    const bool q=p.family==ShellBindingFamily::Qeph;
    const auto count=q?binding.qeph_count():binding.t3_count();
    auto& seen=q?qseen:tseen;
    if(p.family_index>=count||seen[p.family_index])
      return Error(Status::InvalidParent,"Repeated or out-of-range native family parent index",i,p.family);
    const auto source_id=q?binding.qeph_source_id(p.family_index):binding.t3_source_id(p.family_index);
    if(p.source_parent_id!=source_id)
      return Error(Status::IdentityMismatch,"Source parent ID differs from the exact native family order",i,p.family);
    for(std::size_t prior=0;prior<i;++prior) {
      const auto& old=out.parents[prior].declaration;
      if(old.source_part_id==p.source_part_id&&(old.material_id!=p.material_id||old.section_id!=p.section_id))
        return Error(Status::IdentityMismatch,"One source part cannot have conflicting material/section assignments",i,p.family);
    }
    const auto mi=Find(out.materials,out.material_count,p.material_id,[](const auto& x){return x.declaration.material_id;});
    const auto si=Find(out.sections,out.section_count,p.section_id,[](const auto& x){return x.section_id;});
    if(mi==NoShellBindingNode||si==NoShellBindingNode)
      return Error(Status::InvalidParent,"Parent references an absent material or section",i,p.family);
    const auto& m=out.materials[mi].declaration; const auto& s=out.sections[si];
    const bool matches=q? shell_plasticity_binding_detail::Matches(binding.qeph_reference(p.family_index).input,m,s):
      shell_plasticity_binding_detail::Matches(binding.t3_reference(p.family_index).input,m,s);
    if(!matches) return Error(Status::IdentityMismatch,"Parent material/thickness bits differ from its native reference",i,p.family);
    out.parents[i]={p,mi,si}; (q?out.qeph_parent:out.t3_parent)[p.family_index]=i;
    seen[p.family_index]=true; materials[mi]=true; sections[si]=true;
  }
  for(std::size_t i=0;i<out.qeph_count;++i) if(!qseen[i]) return Error(Status::InvalidParent,"Missing QEPH parent",i,ShellBindingFamily::Qeph);
  for(std::size_t i=0;i<out.t3_count;++i) if(!tseen[i]) return Error(Status::InvalidParent,"Missing T3 parent",i,ShellBindingFamily::T3);
  for(std::size_t i=0;i<out.material_count;++i) if(!materials[i]) return Error(Status::UnreferencedDeclaration,"Unreferenced material",i);
  for(std::size_t i=0;i<out.section_count;++i) if(!sections[i]) return Error(Status::UnreferencedDeclaration,"Unreferenced section",i);
  out.parent_count=input.parent_count; return {};
}
} // namespace tl::fea
