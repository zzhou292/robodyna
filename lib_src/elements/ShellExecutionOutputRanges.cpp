// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellExecutionOutputRanges.h"
#include "ShellCatalogCurveView.h"
#include "../solvers/NodalTrialIdentity.h"

namespace tl::fea::shell_execution_detail {
namespace {
template<class T> bool Range(const void* output,std::size_t bytes,const T* source,
    std::size_t count=1) noexcept {
  return !count || trial_identity::Disjoint(output,bytes,source,count*sizeof(T));
}
template<class View> bool ViewRange(const void* output,std::size_t bytes,View source) noexcept {
  return Range(output,bytes,source.data(),source.size());
}
bool TopologyRanges(const rigid::NodalRigidPartTopology& source,
    const void* output,std::size_t bytes) noexcept {
  return Range(output,bytes,&source) &&
      Range(output,bytes,source.parts(),source.part_count()) &&
      Range(output,bytes,source.extras(),source.extra_count()) &&
      Range(output,bytes,source.merges(),source.merge_count()) &&
      Range(output,bytes,source.roots(),source.root_count()) &&
      Range(output,bytes,source.original_members(),source.member_count()) &&
      Range(output,bytes,source.root_members(),source.member_count()) &&
      Range(output,bytes,source.expected_members(),source.member_count()) &&
      Range(output,bytes,source.other_rigid_members(),source.other_rigid_member_count());
}
}
bool OutputDisjoint(const ShellExecutionBinding& execution,
    const void* output,std::size_t bytes) noexcept {
  if (!execution.prepared() || !Range(output,bytes,&execution) ||
      !ViewRange(output,bytes,execution.parents())) return false;
  const auto& catalog=*execution.catalog();
  const shell_batch_plasticity_detail::CatalogCurveView curves(catalog);
  const auto& inventory=catalog.inventory().words();
  if (!Range(output,bytes,&catalog) || !Range(output,bytes,curves.x,curves.count) ||
      !Range(output,bytes,curves.y,curves.count) ||
      !Range(output,bytes,inventory.data(),inventory.size())) return false;
  // The failure catalog can own equivalent, independent declaration backing.
  for (std::size_t row=0;row<catalog.parent_count();++row) {
    if (!Range(output,bytes,catalog.parent(row))) return false;
  }
  const auto& binding=*execution.rigid();
  return OutputDisjoint(binding,output,bytes);
}
bool OutputDisjoint(const NodalRigidAssemblyBinding& binding,
    const void* output,std::size_t bytes) noexcept {
  if (!binding.prepared()) return false;
  const auto& parts=*binding.parts();
  return Range(output,bytes,&binding) &&
      ViewRange(output,bytes,binding.groups()) &&
      ViewRange(output,bytes,binding.members()) &&
      Range(output,bytes,&parts) &&
      ViewRange(output,bytes,parts.members()) &&
      ViewRange(output,bytes,parts.original_bodies()) &&
      ViewRange(output,bytes,parts.roots()) &&
      TopologyRanges(*parts.topology(),output,bytes) &&
      Range(output,bytes,binding.coefficients()) &&
      Range(output,bytes,parts.coefficients());
}
} // namespace tl::fea::shell_execution_detail
