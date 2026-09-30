// SPDX-License-Identifier: AGPL-3.0-or-later
#include "model/Internal.h"
#include <new>
#include <stdexcept>
namespace tl::fea::beam18 {
struct Model::Impl {
  explicit Impl(const NodalNodeDomain& value):domain(value) {}
  NodalNodeDomain domain;
  util::HostArena arena;
  Parent* parents=nullptr;
  MaterialRecord* materials=nullptr;
  std::size_t parent_count=0,material_count=0,retained=0,startup=0;
};
ModelReport Model::Initialize(const NodalNodeDomain& domain,ModelInput input,ModelLimits limits) noexcept try {
  using namespace model_detail;
  if(impl_) return {S::AlreadyInitialized,"Beam model is immutable"};
  auto result=Count(domain,input,limits,sizeof(Impl)); if(!result) return result;
  Index elements,materials;
  elements.Prepare(input.parents.size(),[&](std::size_t p){return input.parents[p].reference.input().source_element_id;});
  materials.Prepare(input.parents.size(),[&](std::size_t p){return input.parents[p].reference.input().source_material_id;});
  Plan plan(limits.max_host_bytes);
  result=Inventory(input,limits,elements,materials,plan); if(!result) return result;
  result=Budget(domain,input,limits,sizeof(Impl),plan); if(!result) return result;
  auto next=std::make_shared<Impl>(domain);
  if(!next->arena.Initialize(plan.arena.bytes())) return {S::ResourceLimit,"Beam model arena allocation failed"};
  next->parents=next->arena.Construct<Parent>(plan.parents);
  next->materials=next->arena.Construct<MaterialRecord>(plan.materials);
  auto* curves=next->arena.Construct<double>(plan.curves);
  if(!next->parents||!next->materials||!curves) return {S::ResourceLimit,"Beam model arena layout failed"};
  std::size_t material_count=0;
  for(std::size_t p=0;p<input.parents.size();++p) {
    const auto& source=input.parents[p]; auto& parent=next->parents[p];
    result=BindParent(domain,source.reference,parent,p); if(!result) return result;
    const auto mid=source.reference.input().source_material_id;
    const auto first=materials.First(mid);
    if(first==p) {
      parent.material_index=material_count++;
      auto& material=next->materials[parent.material_index]; material.source_material_id=mid;
      if(!CopyMaterial(source.material,curves,material.value))
        return {S::InvalidInput,"Owned beam material failed exact preparation",p};
    } else parent.material_index=next->parents[first].material_index;
  }
  next->parent_count=input.parents.size(); next->material_count=material_count;
  next->retained=plan.retained; next->startup=plan.startup;
  impl_=std::move(next); return {};
} catch(const std::bad_alloc&) {return {ModelStatus::ResourceLimit,"Beam model allocation failed"};}
catch(const std::length_error&) {return {ModelStatus::ResourceLimit,"Beam model allocation size overflow"};}
util::ConstView<Parent> Model::parents() const noexcept {
  static const Parent empty; return {impl_?impl_->parents:&empty,impl_?impl_->parent_count:0};
}
util::ConstView<MaterialRecord> Model::materials() const noexcept {
  static const MaterialRecord empty; return {impl_?impl_->materials:&empty,impl_?impl_->material_count:0};
}
const NodalNodeDomain* Model::domain() const noexcept {return impl_?&impl_->domain:nullptr;}
ModelProfile Model::profile() const noexcept {return impl_?ModelProfile::CircularFourPointLaw44V1:ModelProfile::Unspecified;}
std::uint64_t Model::source_instance_id() const noexcept {return impl_?impl_->domain.source_instance_id():0;}
bool Model::Endpoint(std::size_t p,unsigned local,EndpointContribution& output) const noexcept {
  if(!impl_||p>=impl_->parent_count||local>=2) return false;
  const auto& row=impl_->parents[p]; const auto& source=row.reference.input();
  output={source.source_element_id,source.source_part_id,source.source_section_id,source.source_material_id,
      source.source_node_id[local],row.domain_nodes[local],row.reference.endpoint()}; return true;
}
bool Model::SharesStorage(const Model& other) const noexcept {return impl_&&impl_==other.impl_;}
bool Model::Matches(const Model& other) const noexcept {
  if(!impl_||!other.impl_) return false;
  if(impl_==other.impl_) return true;
  if(!impl_->domain.Matches(other.impl_->domain)||impl_->parent_count!=other.impl_->parent_count||
      impl_->material_count!=other.impl_->material_count) return false;
  for(std::size_t p=0;p<impl_->parent_count;++p) {
    const auto& a=impl_->parents[p]; const auto& b=other.impl_->parents[p];
    if(a.material_index!=b.material_index||a.domain_nodes[0]!=b.domain_nodes[0]||a.domain_nodes[1]!=b.domain_nodes[1]||
        !model_detail::SameReference(a.reference,b.reference)) return false;
  }
  for(std::size_t m=0;m<impl_->material_count;++m)
    if(impl_->materials[m].source_material_id!=other.impl_->materials[m].source_material_id||
        !model_detail::SameMaterial(impl_->materials[m].value,other.impl_->materials[m].value)) return false;
  return true;
}
std::size_t Model::owned_payload_bytes() const noexcept {return impl_?impl_->retained:sizeof(Model);}
std::size_t Model::startup_payload_bytes() const noexcept {return impl_?impl_->startup:sizeof(Model);}
} // namespace tl::fea::beam18
