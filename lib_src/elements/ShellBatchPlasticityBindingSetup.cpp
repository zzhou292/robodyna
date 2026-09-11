#include "ShellBatchPlasticityBindingInternal.h"
#include "ShellPlasticityCatalogScratch.h"
#include "ShellBindingValues.h"
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
    data_.qeph_parent.backing_bytes()+data_.t3_parent.backing_bytes()+data_.qbat_parent.backing_bytes();
}
std::size_t ShellBatchPlasticityBinding::startup_scratch_bytes() const noexcept {
  return prepared_?sizeof(Data)+CatalogScratch::Bytes(data_.parent_count,data_.qeph_count,
      data_.t3_count,data_.qbat_count):0;
}
ShellPlasticityBindingReport ShellBatchPlasticityBinding::Initialize(
    const ShellBatchBinding& binding,const ShellBatchPlasticityBindingInput& input,
    const ShellHostBindingLimits& limits) noexcept {
  if(prepared_) return Error(Status::AlreadyInitialized,"Plasticity binding is immutable after preparation");
  if(binding.qbat_count()!=0)
    return Error(Status::InvalidInput,"QBAT requires an explicit complete formulation catalog");
  if(!binding.prepared()||(input.curve_count&&!input.curves)||!input.materials||!input.sections||!input.parents||
     !input.material_count||!input.section_count||!input.parent_count)
    return Error(Status::InvalidInput,"Complete prepared binding and explicit nonempty catalog ranges are required");
  for(auto count:{input.curve_count,input.material_count,input.section_count,input.parent_count})
    if(count>MaxShellHostParents||count>limits.max_parents)
      return Error(Status::ResourceLimit,"Plasticity catalog exceeds host parent admission");
  if(limits.max_parents>MaxShellHostParents||limits.max_nodes>MaxShellHostNodes||binding.node_count()>limits.max_nodes)
    return Error(Status::ResourceLimit,"Plasticity catalog exceeds host node admission");
  // This legacy overload historically ignored the newer binding scratch cap.
  // Preserve that behavior; explicit catalog scratch admission belongs to the
  // named API. Counts/limits above reject vehicle bindings before fixed scratch.
  return InitializeCatalog(binding,input,{limits.max_parents,limits.max_nodes,
    limits.max_parents,limits.max_owned_bytes,MaxVehiclePlasticityCatalogScratchBytes});
}
ShellPlasticityBindingReport ShellBatchPlasticityBinding::InitializeCatalog(
    const ShellBatchBinding& binding,const ShellBatchPlasticityBindingInput& input,
    const ShellPlasticityCatalogLimits& limits) noexcept {
  return InitializeCatalogImpl(binding,input,limits,false);
}
ShellPlasticityBindingReport ShellBatchPlasticityBinding::InitializeCatalogImpl(
    const ShellBatchBinding& binding,const ShellBatchPlasticityBindingInput& input,
    const ShellPlasticityCatalogLimits& limits,bool heterogeneous,bool formulations) noexcept {
  if(prepared_) return Error(Status::AlreadyInitialized,"Plasticity binding is immutable after preparation");
  if(binding.qbat_count()!=0&&!formulations)
    return Error(Status::InvalidInput,"QBAT requires an explicit complete formulation catalog");
  if(formulations&&(!heterogeneous||binding.qbat_count()==0))
    return Error(Status::InvalidInput,"Explicit formulation catalog requires QBAT and typed sections");
  if(!binding.prepared()||(input.curve_count&&!input.curves)||!input.materials||!input.sections||!input.parents||
     !input.material_count||!input.section_count||!input.parent_count)
    return Error(Status::InvalidInput,"Complete prepared binding and explicit nonempty catalog ranges are required");
  if(limits.max_parents>MaxVehiclePlasticityCatalogParents||limits.max_nodes>MaxVehiclePlasticityCatalogNodes||
     limits.max_definitions>MaxPlasticityCatalogDefinitions||
     input.parent_count>limits.max_parents||binding.node_count()>limits.max_nodes)
    return Error(Status::ResourceLimit,"Plasticity catalog exceeds explicit host admission");
  for(auto count:{input.curve_count,input.material_count,input.section_count})
    if(count>limits.max_definitions)
      return Error(Status::ResourceLimit,"Plasticity catalog exceeds definition admission");
  if(input.parent_count!=binding.qeph_count()+binding.t3_count()+binding.qbat_count())
    return Error(Status::InvalidParent,"Plasticity mapping must cover every native collection parent");
  const auto bytes=sizeof(*this)+binding.inventory().backing_bytes()+
    decltype(data_.curves)::ExtraBytes(input.curve_count)+
    decltype(data_.materials)::ExtraBytes(input.material_count)+
    decltype(data_.sections)::ExtraBytes(input.section_count)+
    decltype(data_.parents)::ExtraBytes(input.parent_count)+
    decltype(data_.qeph_parent)::ExtraBytes(binding.qeph_count())+
    decltype(data_.t3_parent)::ExtraBytes(binding.t3_count())+
    decltype(data_.qbat_parent)::ExtraBytes(binding.qbat_count());
  if(bytes>limits.max_owned_bytes)
    return Error(Status::ResourceLimit,"Owned host plasticity catalog exceeds byte admission");
  // Bounds above limit every multiplication/addition to <256 MiB. The staging
  // object and private temporary indexes are separate from retained backing.
  const auto scratch=sizeof(Data)+CatalogScratch::Bytes(input.parent_count,binding.qeph_count(),
      binding.t3_count(),binding.qbat_count());
  if(scratch>limits.max_startup_scratch_bytes)
    return Error(Status::ResourceLimit,"Plasticity catalog startup scratch exceeds byte admission");
  if(formulations&&(!shell_binding_detail::ValidRange(input.curves,input.curve_count)||
      !shell_binding_detail::ValidRange(input.materials,input.material_count)||
      !shell_binding_detail::ValidRange(input.sections,input.section_count)||
      !shell_binding_detail::ValidRange(input.parents,input.parent_count)))
    return Error(Status::InvalidInput,"Formulation catalog ranges are misaligned or overflowing");
  // All range counts and the TOTAL pool size are checked before any curve copy.
  std::size_t points=0;
  for(std::size_t i=0;i<input.curve_count;++i) {
    const auto& c=input.curves[i];
    if(!c.curve_id||!c.curve.plastic_strain||!c.curve.yield_stress_pa||c.curve.count<2)
      return Error(Status::InvalidCurve,"Curve requires a nonzero ID and at least two supplied points",i);
    if(c.curve.count>MaxShellPlasticityCurvePoints-points)
      return Error(Status::ResourceLimit,"Complete plasticity curve pool exceeds 1024 points",i);
    if(formulations&&(!shell_binding_detail::ValidRange(c.curve.plastic_strain,c.curve.count)||
        !shell_binding_detail::ValidRange(c.curve.yield_stress_pa,c.curve.count)))
      return Error(Status::InvalidCurve,"Formulation curve range is misaligned or overflowing",i);
    points+=c.curve.count;
  }
  try { return Build(binding,input,heterogeneous,formulations); }
  catch(const std::bad_alloc&) { return Error(Status::ResourceLimit,"Plasticity catalog startup allocation failed"); }
}
ShellPlasticityBindingReport ShellBatchPlasticityBinding::Build(
    const ShellBatchBinding& binding,const ShellBatchPlasticityBindingInput& input,
    bool heterogeneous,bool formulations) {
  Data staged;
  staged.heterogeneous=heterogeneous;
  staged.formulations=formulations;
  staged.curves.Resize(input.curve_count); staged.materials.Resize(input.material_count);
  staged.sections.Resize(input.section_count); staged.parents.Resize(input.parent_count);
  staged.qeph_parent.Resize(binding.qeph_count()); staged.t3_parent.Resize(binding.t3_count());
  staged.qbat_parent.Resize(binding.qbat_count());
  staged.inventory=binding.inventory(); staged.qeph_count=binding.qeph_count(); staged.t3_count=binding.t3_count();
  staged.qbat_count=binding.qbat_count();
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
  std::array<bool,MaxPlasticityCatalogDefinitions> used{};
  for(std::size_t i=0;i<input.material_count;++i) {
    const auto& m=input.materials[i];
    if(!m.material_id||Find(out.materials,i,m.material_id,
        [](const auto& x){return x.declaration.material_id;})!=NoShellBindingNode)
      return Error(Status::InvalidMaterial,"Missing or duplicate material ID",i);
    auto& target=out.materials[i];
    if(m.law==ShellSectionLaw::LayeredLaw1Nip3) {
      material::ShellElasticLaw1PointParameters elastic;
      if(!out.heterogeneous||m.curve_id||m.hardening!=material::ShellPlasticityHardeningKind::Tabulated||
         m.continuation!=material::ShellPlasticityCurveContinuation::StrictDomain||
         !Same(m.rate,material::TabulatedShellPlasticityRate{})||
         !Same(m.linear.initial_yield_pa,0.)||!Same(m.linear.tangent_modulus_pa,0.)||
         !material::PrepareShellElasticLaw1Point(m.young_pa,m.poisson_ratio,m.density_kg_m3,elastic))
        return Error(Status::InvalidMaterial,"Layered LAW1 requires explicit section mode, elastic coefficients and no plastic controls",i);
      target.declaration=m;target.curve_index=NoShellBindingNode;
      continue;
    }
    if(m.law!=ShellSectionLaw::LayeredLaw44Nip3)
      return Error(Status::InvalidMaterial,"Unknown layered shell material law",i);
    if(m.hardening==material::ShellPlasticityHardeningKind::LinearLaw44) {
      if(m.curve_id||m.continuation!=material::ShellPlasticityCurveContinuation::StrictDomain||
          material::PrepareLinearLaw44ShellPlasticity(m.young_pa,m.poisson_ratio,m.density_kg_m3,
          m.linear,m.rate,target.coefficients)!=sections::PointStatus::Ok)
        return Error(Status::InvalidMaterial,"Analytic LAW44 requires no curve, valid SIGY/ETAN and positive source rate",i);
      target.declaration=m; target.curve_index=NoShellBindingNode;
      continue;
    }
    if(m.hardening!=material::ShellPlasticityHardeningKind::Tabulated||!m.curve_id||
       m.linear.initial_yield_pa!=0||m.linear.tangent_modulus_pa!=0)
      return Error(Status::InvalidMaterial,"Tabulated material requires a curve and no analytic declaration",i);
    const auto ci=Find(out.curves,out.curve_count,m.curve_id,[](const auto& x){return x.id;});
    if(ci==NoShellBindingNode) return Error(Status::InvalidMaterial,"Material references an absent curve",i);
    const auto& c=out.curves[ci];
    const material::TabulatedShellPlasticityCurve curve{out.curve_x.data()+c.offset,out.curve_y.data()+c.offset,
      static_cast<std::uint32_t>(c.count)};
    if(material::PrepareTabulatedShellPlasticity(m.young_pa,m.poisson_ratio,m.density_kg_m3,
        curve,m.rate,m.continuation,target.coefficients)!=sections::PointStatus::Ok)
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
    const bool layered=s.formulation==ShellSectionFormulation::LayeredNip3&&s.through_thickness_points==3;
    const bool one_point=out.heterogeneous&&s.formulation==ShellSectionFormulation::OneThicknessPoint&&
        s.through_thickness_points==1;
    if(!s.section_id||!tl::math::Finite(s.thickness_m)||s.thickness_m<=0||(!layered&&!one_point)||
        Find(out.sections,i,s.section_id,[](const auto& x){return x.section_id;})!=NoShellBindingNode)
      return Error(Status::InvalidSection,"Section requires unique ID, positive thickness and an explicit supported point count",i);
    out.sections[i]=s;
  }
  out.section_count=input.section_count; return {};
}

} // namespace tl::fea
