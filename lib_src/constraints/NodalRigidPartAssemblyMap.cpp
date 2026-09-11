// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidPartAssemblyInternal.h"

namespace tl::fea::rigid::part_assembly_detail {
namespace {
bool Covered(const CoefficientOccurrences& n) noexcept {
  return n.qeph||n.t3||n.qbat||n.type25||n.type13||n.element_mass;
}
Report MapRange(Storage& s,std::size_t part,std::size_t offset,std::size_t count) {
  const auto& domain=*s.coefficients.domain();
  for(std::size_t n=offset;n<offset+count;++n) {
    const auto index=domain.Find(s.topology.original_members()[n]);
    if(index==SIZE_MAX)
      return Fail(S::IdentityMismatch,"Rigid member NID is absent from the supplied domain",part,n);
    if(!Covered(s.coefficients.nodes()[index].occurrences))
      return Fail(S::MissingCoefficient,"Rigid member has no admitted coefficient producer",part,n);
    s.members[n]={index,part};
  }
  std::sort(s.members+offset,s.members+offset+count,[&](const auto& a,const auto& b) {
    return domain.nodes()[a.domain_node].source_id<domain.nodes()[b.domain_node].source_id;
  });
  return {};
}
Report PrepareOriginal(Storage& s,std::size_t p,const AssemblyMassPoint* points) {
  const auto& part=s.topology.parts()[p];
  AssemblyPrimary primary;
  // Native primary regularizers, converted once from declared source units.
  primary.mass=1e-20*s.units.mass_to_kg;
  primary.inertia=primary.mass*s.units.length_to_m*s.units.length_to_m;
  for(std::size_t i=0;i<part.member_count;++i) {
    const auto x=points[part.member_offset+i].position;
    primary.position.x=primary.position.x+x.x;
    primary.position.y=primary.position.y+x.y;
    primary.position.z=primary.position.z+x.z;
  }
  if(!Finite(primary.position)) return Fail(S::NonfiniteResult,"Rigid PART primary mean overflow",p);
  primary.position.x=primary.position.x/double(part.member_count);
  primary.position.y=primary.position.y/double(part.member_count);
  primary.position.z=primary.position.z/double(part.member_count);
  AssemblyBodyInput input;
  input.primary=primary;
  input.part=points+part.member_offset;
  input.part_count=part.member_count;
  input.source_units=s.units;
  if(part.extra_row!=SIZE_MAX) {
    const auto& extra=s.topology.extras()[part.extra_row];
    input.extra=points+extra.member_offset;
    input.extra_count=extra.member_count;
  }
  auto& result=s.originals[p];
  const auto status=PrepareAssemblyRawBody(input,result.raw);
  if(status!=AssemblyValueStatus::Success)
    return Fail(status==AssemblyValueStatus::NonfiniteResult?S::NonfiniteResult:S::InvalidInput,
                "Rigid raw PART/extra preparation failed",p);
  result.primary=primary;
  return {};
}
}
Report MapAndAssemble(Storage& s) {
  const auto& t=s.topology;
  for(std::size_t p=0;p<t.part_count();++p) {
    const auto& part=t.parts()[p];
    auto report=MapRange(s,p,part.member_offset,part.member_count);
    if(!report) return report;
    if(part.extra_row!=SIZE_MAX) {
      const auto& extra=t.extras()[part.extra_row];
      report=MapRange(s,p,extra.member_offset,extra.member_count);
      if(!report) return report;
    }
  }
  auto points=std::make_unique<AssemblyMassPoint[]>(t.member_count());
  for(std::size_t n=0;n<t.member_count();++n) {
    const auto index=s.members[n].domain_node;
    const auto& value=s.coefficients.nodes()[index].coefficients;
    points[n]={s.coefficients.domain()->nodes()[index].position,value.mass,value.isotropic_inertia};
    s.lookup[n]=n;
  }
  std::sort(s.lookup,s.lookup+t.member_count(),[&](std::size_t a,std::size_t b) {
    return s.members[a].domain_node<s.members[b].domain_node;
  });
  for(std::size_t p=0;p<t.part_count();++p) {
    const auto report=PrepareOriginal(s,p,points.get());
    if(!report) return report;
  }
  for(std::size_t r=0;r<t.root_count();++r)
    s.roots[r].value.raw=s.originals[t.roots()[r].part_index].raw;
  util::SourceIdentityIndex<0> parts;
  parts.Prepare(t.part_count(),[&](std::size_t p){return t.parts()[p].source_part_id;});
  // Source-directed RAW merges precede every eigen correction. Original body
  // records remain independently visible, including each generated primary.
  for(std::size_t m=0;m<t.merge_count();++m) {
    const auto parent=parts.First(t.merges()[m].parent_part_id);
    const auto child=parts.First(t.merges()[m].child_part_id);
    auto& root=s.roots[t.parts()[parent].root_index].value.raw;
    AssemblyRawBody merged;
    if(MergeAssemblyRawBodies(root,s.originals[child].raw,merged)!=AssemblyValueStatus::Success)
      return Fail(S::NonfiniteResult,"Rigid directed raw-body merge failed",parent);
    root=merged;
  }
  for(std::size_t r=0;r<t.root_count();++r) {
    AssemblyFinalBody final;
    const auto report=FinalizeAssemblyRawBody(s.roots[r].value.raw,final);
    if(!report) return Fail(S::InertiaFailure,report.message,t.roots()[r].part_index);
    s.roots[r].value=final;
  }
  return {};
}
} // namespace tl::fea::rigid::part_assembly_detail
