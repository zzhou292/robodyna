#include "ShellBatchPlasticityBindingInternal.h"
#include "ShellPlasticityCatalogScratch.h"

namespace tl::fea {
using namespace shell_plasticity_binding_detail;

ShellPlasticityBindingReport ShellBatchPlasticityBinding::BindParents(const ShellBatchBinding& binding,
    const ShellBatchPlasticityBindingInput& input,Data& out) {
  CatalogScratch scratch;
  scratch.parts.Prepare(input.parent_count,[&](std::size_t i){return input.parents[i].source_part_id;});
  scratch.materials.Prepare(out.material_count,[&](std::size_t i){return out.materials[i].declaration.material_id;});
  scratch.sections.Prepare(out.section_count,[&](std::size_t i){return out.sections[i].section_id;});
  scratch.qeph.Resize(out.qeph_count); scratch.t3.Resize(out.t3_count);
  scratch.qbat.Resize(out.qbat_count);
  auto& qseen=scratch.qeph; auto& tseen=scratch.t3;
  auto& materials=scratch.material_seen; auto& sections=scratch.section_seen;
  for(std::size_t i=0;i<input.parent_count;++i) {
    const auto& p=input.parents[i];
    if(!ValidShellParentExecution(p.execution)||
        (p.execution.policy!=ShellParentExecutionPolicy::FromSection&&!out.execution))
      return Error(Status::InvalidParent,"Resolved parent execution requires explicit execution catalog and valid policy",i,p.family);
    if(!p.source_parent_id||!p.source_part_id||!p.material_id||!p.section_id||
       (p.family!=ShellBindingFamily::Qeph&&p.family!=ShellBindingFamily::T3&&
        !(out.formulations&&p.family==ShellBindingFamily::Qbat)))
      return Error(Status::InvalidParent,"Explicit source parent/part/material/section IDs and family are required",i,p.family);
    std::size_t count=0;
    bool* seen=nullptr;
    std::size_t* rows=nullptr;
    ShellSectionCounts* laws=nullptr;
    switch(p.family) {
      case ShellBindingFamily::Qeph:
        count=out.qeph_count;
        seen=qseen.data();
        rows=out.qeph_parent.data();
        laws=&out.qeph_laws;
        break;
      case ShellBindingFamily::T3:
        count=out.t3_count;
        seen=tseen.data();
        rows=out.t3_parent.data();
        laws=&out.t3_laws;
        break;
      case ShellBindingFamily::Qbat:
        count=out.qbat_count;
        seen=scratch.qbat.data();
        rows=out.qbat_parent.data();
        laws=&out.qbat_laws;
        break;
      default:
        return Error(Status::InvalidParent,"Unknown formulation family",i,p.family);
    }
    if(p.family_index>=count||seen[p.family_index])
      return Error(Status::InvalidParent,"Repeated or out-of-range native family parent index",i,p.family);
    const auto source_id=p.family==ShellBindingFamily::Qeph?binding.qeph_source_id(p.family_index):
        p.family==ShellBindingFamily::T3?binding.t3_source_id(p.family_index):binding.qbat_source_id(p.family_index);
    if(p.source_parent_id!=source_id)
      return Error(Status::IdentityMismatch,"Source parent ID differs from the exact native family order",i,p.family);
    // Earlier successful occurrences of this PID necessarily match its first
    // assignment. Compare only that prior occurrence, preserving the original
    // source-order first failure while avoiding an all-parent scan.
    const auto prior=scratch.parts.First(p.source_part_id);
    if(prior<i) {
      const auto& old=out.parents[prior].declaration;
      if(old.material_id!=p.material_id||old.section_id!=p.section_id||
        !SameShellParentExecution(old.execution,p.execution))
        return Error(Status::IdentityMismatch,"One source part cannot have conflicting material/section assignments",i,p.family);
    }
    const auto mi=scratch.materials.First(p.material_id);
    const auto si=scratch.sections.First(p.section_id);
    if(mi==NoShellBindingNode||si==NoShellBindingNode)
      return Error(Status::InvalidParent,"Parent references an absent material or section",i,p.family);
    const auto& m=out.materials[mi].declaration; const auto& s=out.sections[si];
    bool matches=false;
    auto placement=ShellReferencePlacement::Centered;
    if(p.family==ShellBindingFamily::Qeph) {
      const auto& reference=binding.qeph_reference(p.family_index).input;
      matches=shell_plasticity_binding_detail::Matches(reference,m,s);
      placement=reference.placement;
    } else if(p.family==ShellBindingFamily::T3) {
      const auto& reference=binding.t3_reference(p.family_index).input;
      matches=shell_plasticity_binding_detail::Matches(reference,m,s);
      placement=reference.placement;
    } else {
      const auto& reference=binding.qbat_reference(p.family_index).input();
      matches=shell_plasticity_binding_detail::Matches(reference.quadrilateral,m,s)&&
          Same(reference.initial_a11_pa,out.materials[mi].coefficients.a11);
      placement=reference.quadrilateral.placement;
    }
    if(!matches) return Error(Status::IdentityMismatch,"Parent material/thickness bits differ from its native reference",i,p.family);
    const bool global=p.execution.policy==ShellParentExecutionPolicy::GlobalLaw1Npt0;
    if(global&&(m.law!=ShellSectionLaw::LayeredLaw1Nip3||p.family==ShellBindingFamily::Qbat||
        placement!=ShellReferencePlacement::Centered||s.formulation!=ShellSectionFormulation::LayeredNip3||
        s.through_thickness_points!=3))
      return Error(Status::InvalidSection,"Global LAW1 execution requires centered QEPH/T3 with the qualified raw NIP3 elastic source",i,p.family);
    const bool rigid=m.law==ShellSectionLaw::RigidSkin;
    if(rigid!=(s.formulation==ShellSectionFormulation::Nonconstitutive)||
        (rigid&&(!out.execution||p.family==ShellBindingFamily::Qbat)))
      return Error(Status::InvalidSection,"Rigid skin requires QEPH/T3 and an explicit zero-point section",i,p.family);
    const bool one_point=s.formulation==ShellSectionFormulation::OneThicknessPoint;
    if(one_point&&(p.family==ShellBindingFamily::Qeph||m.law!=ShellSectionLaw::LayeredLaw44Nip3||
        placement!=ShellReferencePlacement::Centered))
      return Error(Status::InvalidSection,out.formulations?
          "One thickness point requires centered T3/QBAT and LAW44":
          "One thickness point requires centered T3 and LAW44",i,p.family);
    if(p.family==ShellBindingFamily::Qbat&&!one_point)
      return Error(Status::InvalidSection,"QBAT requires the explicit shared one-thickness-point section",i,p.family);
    out.parents[i]={p,mi,si};
    rows[p.family_index]=i;
    if(rigid) ++laws->rigid_skin;
    else if(m.law==ShellSectionLaw::LayeredLaw1Nip3) {
      ++laws->law1;if(global)++laws->law1_global_npt0;
    }
    else if(m.law==ShellSectionLaw::LayeredLaw44Nip3) {
      ++laws->law44;
      if(one_point&&p.family==ShellBindingFamily::T3) ++laws->law44_nip1;
      if(p.family==ShellBindingFamily::Qbat) ++laws->law44_qbat;
    } else return Error(Status::InvalidMaterial,"Unknown shell material kind",i,p.family);
    seen[p.family_index]=true; materials[mi]=true; sections[si]=true;
  }
  for(std::size_t i=0;i<out.qeph_count;++i) if(!qseen[i]) return Error(Status::InvalidParent,"Missing QEPH parent",i,ShellBindingFamily::Qeph);
  for(std::size_t i=0;i<out.t3_count;++i) if(!tseen[i]) return Error(Status::InvalidParent,"Missing T3 parent",i,ShellBindingFamily::T3);
  for(std::size_t i=0;i<out.qbat_count;++i) if(!scratch.qbat[i]) return Error(Status::InvalidParent,"Missing QBAT parent",i,ShellBindingFamily::Qbat);
  for(std::size_t i=0;i<out.material_count;++i) if(!materials[i]) return Error(Status::UnreferencedDeclaration,"Unreferenced material",i);
  for(std::size_t i=0;i<out.section_count;++i) if(!sections[i]) return Error(Status::UnreferencedDeclaration,"Unreferenced section",i);
  out.parent_count=input.parent_count; return {};
}
} // namespace tl::fea
