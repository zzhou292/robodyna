// SPDX-License-Identifier: AGPL-3.0-or-later
#include "model/Impl.h"
#include <algorithm>
#include <new>
#include <stdexcept>

namespace tl::fea::solids {
ModelReport Model::Initialize(const NodalNodeDomain& domain,ModelInput input,ModelLimits limits) noexcept try {
  using namespace model_detail;
  if(impl_)return Error(ModelStatus::AlreadyInitialized,"Solid mechanics model is immutable",input);
  Layout layout;
  auto report=Preflight(domain,input,limits,sizeof(Model)+sizeof(Impl)+64,layout);
  if(!report)return report;
  Scratch scratch;
  if(!scratch.arena.Initialize(layout.scratch_arena_bytes))
    return Error(ModelStatus::ResourceLimit,"Solid reference scratch allocation failed",input);
  scratch.reference18=scratch.arena.Construct<solid18::Reference>(layout.reference18);
  scratch.reference24=scratch.arena.Construct<solid24::Reference>(layout.reference24);
  scratch.reference6z=scratch.arena.Construct<solid6z::Reference>(layout.reference6z);
  scratch.reference44=scratch.arena.Construct<solid18::law44::Reference>(layout.reference44);
  scratch.reference90=scratch.arena.Construct<solid18::total_strain::Reference>(layout.reference90);
  scratch.material_indices=scratch.arena.Construct<std::size_t>(layout.material_indices);
  if(!scratch.reference18 || !scratch.reference24 || !scratch.reference6z || !scratch.reference44 || !scratch.reference90 || !scratch.material_indices)
    return Error(ModelStatus::ResourceLimit,"Solid scratch layout is invalid",input);
  for(std::size_t i=0;i<input.solid18.size();++i)scratch.reference18[i]=input.solid18[i].reference;
  for(std::size_t i=0;i<input.solid24.size();++i)scratch.reference24[i]=input.solid24[i].reference;
  for(std::size_t i=0;i<input.solid6z.size();++i)scratch.reference6z[i]=input.solid6z[i].reference;
  for(std::size_t i=0;i<input.solid18_law44.size();++i)scratch.reference44[i]=input.solid18_law44[i].reference;
  for(std::size_t i=0;i<input.solid18_law90.size();++i)scratch.reference90[i]=input.solid18_law90[i].reference;
  report=PlanMaterials(input,limits,scratch,layout);
  if(!report)return report;
  report=CompleteLayout(input,limits,layout);
  if(!report)return report;
  if(layout.startup_bytes>=limits.max_host_bytes)
    return Error(ModelStatus::ResourceLimit,"No budget remains for solid coefficient identity",input);
  SolidCoefficientLimits coefficient_limits;
  coefficient_limits.max_parents=limits.max_parents;
  coefficient_limits.max_nodes=limits.max_nodes;
  // The embedded contribution handle is already in sizeof(Impl). Everything
  // behind it, including the shared domain, remains charged once by its owner.
  coefficient_limits.max_host_bytes=limits.max_host_bytes-layout.startup_bytes+sizeof(SolidNodeContributions);
  SolidCoefficientInput coefficient_input;
  coefficient_input.source_instance_id=input.source_instance_id;
  coefficient_input.solid18_count=input.solid18.size();
  coefficient_input.solid24_count=input.solid24.size();
  coefficient_input.solid6z_count=input.solid6z.size();
  coefficient_input.solid18=input.solid18.size()?scratch.reference18:nullptr;
  coefficient_input.solid24=input.solid24.size()?scratch.reference24:nullptr;
  coefficient_input.solid6z=input.solid6z.size()?scratch.reference6z:nullptr;
  coefficient_input.law44_count=input.solid18_law44.size();
  coefficient_input.law90_count=input.solid18_law90.size();
  coefficient_input.law44=input.solid18_law44.size()?scratch.reference44:nullptr;
  coefficient_input.law90=input.solid18_law90.size()?scratch.reference90:nullptr;
  coefficient_input.profile=input.profile==ModelProfile::ExtendedLaw44Law90
      ? SolidCoefficientProfile::ExtendedLaw44Law90 : SolidCoefficientProfile::OriginalThreeFamilies;
  SolidNodeContributions coefficients;
  const auto mapped=coefficients.Initialize(domain,coefficient_input,coefficient_limits);
  if(!mapped) {
    auto status=ModelStatus::SourceMismatch;
    if(mapped.status==NodalDomainStatus::ResourceLimit)status=ModelStatus::ResourceLimit;
    if(mapped.status==NodalDomainStatus::DuplicateIdentity)status=ModelStatus::DuplicateIdentity;
    return Error(status,mapped.message,input,mapped.node);
  }
  layout.owned_bytes+=coefficients.owned_payload_bytes()-sizeof(SolidNodeContributions);
  layout.startup_bytes+=coefficients.startup_payload_bytes()-sizeof(SolidNodeContributions);
  auto next=std::make_shared<Impl>(coefficients,input.profile);
  next->layout=layout;
  auto& storage=next->storage;
  if(!storage.arena.Initialize(layout.arena_bytes))
    return Error(ModelStatus::ResourceLimit,"Solid mechanics arena allocation failed",input);
  storage.parent18=storage.arena.Construct<Parent18>(layout.parent18);
  storage.parent24=storage.arena.Construct<Parent24>(layout.parent24);
  storage.parent6z=storage.arena.Construct<Parent6z>(layout.parent6z);
  storage.parent44=storage.arena.Construct<Parent18Law44>(layout.parent44);
  storage.parent90=storage.arena.Construct<Parent18Law90>(layout.parent90);
  storage.material44=storage.arena.Construct<Material44>(layout.material44);
  storage.material90=storage.arena.Construct<Material90>(layout.material90);
  storage.material36=storage.arena.Construct<Material36>(layout.material36);
  storage.material42=storage.arena.Construct<Material42>(layout.material42);
  storage.curves=storage.arena.Construct<double>(layout.curves);
  if(!storage.parent18 || !storage.parent24 || !storage.parent6z ||
      !storage.parent44 || !storage.parent90 || !storage.material44 || !storage.material90 ||
      !storage.material36 || !storage.material42 || !storage.curves)
    return Error(ModelStatus::ResourceLimit,"Solid mechanics arena layout is invalid",input);
  report=CopyMaterials(input,scratch,layout,storage);
  if(!report)return report;
  report=CopyParents(input,scratch,coefficients,storage);
  if(!report)return report;
  control::Limits control_limits;
  control_limits.max_parents=limits.max_parents;control_limits.max_packets=limits.max_parents;
  control_limits.max_host_bytes=layout.control_budget.startup_bytes;
  const auto selected=next->controls.Initialize(coefficients,input.controls,control_limits);
  if(!selected) {
    auto status=ModelStatus::SourceMismatch;
    if(selected.status==control::Status::ResourceLimit)status=ModelStatus::ResourceLimit;
    if(selected.status==control::Status::DuplicateIdentity)status=ModelStatus::DuplicateIdentity;
    if(selected.status==control::Status::InvalidInput||selected.status==control::Status::UnsupportedProfile)status=ModelStatus::InvalidInput;
    return Error(status,selected.message,input,selected.parent);
  }

  impl_=std::move(next);
  return {};
} catch(const std::bad_alloc&) {
  return {ModelStatus::ResourceLimit,"Solid model allocation failed"};
} catch(const std::length_error&) {
  return {ModelStatus::ResourceLimit,"Solid model allocation extent overflow"};
}
} // namespace tl::fea::solids
