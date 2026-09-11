// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellFormulationScope.h"
#include "ShellCatalogCurveView.h"
#include "../assembly/NodalMassBinding.h"
#include "../solvers/NodalTrialIdentity.h"

namespace tl::fea::shell_formulation_detail {
// All source ranges belong to prepared immutable handles. Equivalent catalogs
// may own different backing; validate each participant's actual retained view.
inline bool OutputDisjoint(const ShellFormulationScope& scope,const void* output,std::size_t bytes) noexcept {
  using trial_identity::Disjoint;
  if(!scope.binding||!scope.failure||!scope.failure->catalog()||
      !Disjoint(output,bytes,scope.binding,sizeof(*scope.binding))||
      !Disjoint(output,bytes,scope.failure,sizeof(*scope.failure))||
      !Disjoint(output,bytes,scope.binding->nodes().data(),scope.binding->node_count()*sizeof(ShellBindingNode))) return false;
  const auto& catalog=*scope.failure->catalog();
  const shell_batch_plasticity_detail::CatalogCurveView curves(catalog);
  if(!Disjoint(output,bytes,&catalog,sizeof(catalog))||
      (curves.count&&(!Disjoint(output,bytes,curves.x,curves.count*sizeof(double))||
                     !Disjoint(output,bytes,curves.y,curves.count*sizeof(double))))) return false;
  for(std::size_t parent=0;parent<catalog.parent_count();++parent) {
    if(!Disjoint(output,bytes,catalog.parent(parent),sizeof(ShellPlasticityParentInput))||
        !Disjoint(output,bytes,scope.failure->parent(parent),sizeof(ShellFailureParentInput))) return false;
  }
  for(std::size_t parent=0;parent<scope.binding->qbat_count();++parent) {
    if(!Disjoint(output,bytes,&scope.binding->qbat_reference(parent),sizeof(qbat::Reference))||
        !Disjoint(output,bytes,scope.binding->qbat_nodes(parent).data(),sizeof(std::array<std::size_t,4>))) return false;
  }
  for(std::size_t parent=0;parent<scope.binding->qeph_count();++parent) {
    if(!Disjoint(output,bytes,&scope.binding->qeph_reference(parent),sizeof(qeph::ReferenceData))||
        !Disjoint(output,bytes,scope.binding->qeph_nodes(parent).data(),sizeof(std::array<std::size_t,4>))) return false;
  }
  for(std::size_t parent=0;parent<scope.binding->t3_count();++parent) {
    if(!Disjoint(output,bytes,&scope.binding->t3_reference(parent),sizeof(t3::ReferenceData))||
        !Disjoint(output,bytes,scope.binding->t3_nodes(parent).data(),sizeof(std::array<std::size_t,3>))) return false;
  }
  const auto& inventory=scope.binding->inventory().words();
  if(!Disjoint(output,bytes,inventory.data(),inventory.size()*sizeof(std::uint64_t))) return false;
  return !scope.mass||Disjoint(output,bytes,scope.mass->nodes().data(),scope.mass->node_count()*sizeof(NodalMassNode));
}
} // namespace tl::fea::shell_formulation_detail
