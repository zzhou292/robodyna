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
  auto& qseen=scratch.qeph; auto& tseen=scratch.t3;
  auto& materials=scratch.material_seen; auto& sections=scratch.section_seen;
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
    // Earlier successful occurrences of this PID necessarily match its first
    // assignment. Compare only that prior occurrence, preserving the original
    // source-order first failure while avoiding an all-parent scan.
    const auto prior=scratch.parts.First(p.source_part_id);
    if(prior<i) {
      const auto& old=out.parents[prior].declaration;
      if(old.material_id!=p.material_id||old.section_id!=p.section_id)
        return Error(Status::IdentityMismatch,"One source part cannot have conflicting material/section assignments",i,p.family);
    }
    const auto mi=scratch.materials.First(p.material_id);
    const auto si=scratch.sections.First(p.section_id);
    if(mi==NoShellBindingNode||si==NoShellBindingNode)
      return Error(Status::InvalidParent,"Parent references an absent material or section",i,p.family);
    const auto& m=out.materials[mi].declaration; const auto& s=out.sections[si];
    const bool matches=q? shell_plasticity_binding_detail::Matches(binding.qeph_reference(p.family_index).input,m,s):
      shell_plasticity_binding_detail::Matches(binding.t3_reference(p.family_index).input,m,s);
    if(!matches) return Error(Status::IdentityMismatch,"Parent material/thickness bits differ from its native reference",i,p.family);
    const bool one_point=s.formulation==ShellSectionFormulation::OneThicknessPoint;
    if(one_point&&(q||m.law!=ShellSectionLaw::LayeredLaw44Nip3||
        binding.t3_reference(p.family_index).input.placement!=ShellReferencePlacement::Centered))
      return Error(Status::InvalidSection,"One thickness point requires centered T3 and LAW44",i,p.family);
    out.parents[i]={p,mi,si}; (q?out.qeph_parent:out.t3_parent)[p.family_index]=i;
    auto& laws=q?out.qeph_laws:out.t3_laws;
    if(m.law==ShellSectionLaw::LayeredLaw1Nip3) ++laws.law1;
    else if(m.law==ShellSectionLaw::LayeredLaw44Nip3) {
      ++laws.law44;
      if(one_point) ++laws.law44_nip1;
    } else return Error(Status::InvalidMaterial,"Unknown shell material kind",i,p.family);
    seen[p.family_index]=true; materials[mi]=true; sections[si]=true;
  }
  for(std::size_t i=0;i<out.qeph_count;++i) if(!qseen[i]) return Error(Status::InvalidParent,"Missing QEPH parent",i,ShellBindingFamily::Qeph);
  for(std::size_t i=0;i<out.t3_count;++i) if(!tseen[i]) return Error(Status::InvalidParent,"Missing T3 parent",i,ShellBindingFamily::T3);
  for(std::size_t i=0;i<out.material_count;++i) if(!materials[i]) return Error(Status::UnreferencedDeclaration,"Unreferenced material",i);
  for(std::size_t i=0;i<out.section_count;++i) if(!sections[i]) return Error(Status::UnreferencedDeclaration,"Unreferenced section",i);
  out.parent_count=input.parent_count; return {};
}
} // namespace tl::fea
