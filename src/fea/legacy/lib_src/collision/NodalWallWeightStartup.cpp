#include "NodalWallWeightStartup.h"
#include "Q4ParametricContact.h"

namespace tlfea::contact::nodal_wall_detail {
NodalWallReport PrepareParentWeight(std::uint32_t global_nodes,const NodalWallParentInput& in,
    NodalWallParentWeight* output) noexcept {
  auto failure=Report(NodalWallStatus::InvalidReference);
  if(!output) return failure;
  NodalWallParentWeight out;
    if ((in.q4==nullptr)==(in.t3==nullptr)) return failure;
    if (in.q4) {
      if (!in.q4->prepared() || in.q4->node_count()!=global_nodes || in.q4_parent>=in.q4->parent_count()) return failure;
      const auto& reference=in.q4->parent(in.q4_parent);
      if (!reference.intrinsic.prepared()) return failure;
      const auto& parent=reference.intrinsic.parent();
      out.area=reference.area; out.arity=4; out.family=NodalWallParentFamily::Q4CenterArea;
      out.parent_element_id=parent.parent_element_id; out.parent_face_id=parent.parent_face_id; out.feature_id=parent.feature_id;
      for (unsigned n=0;n<4;++n) out.nodes[n]=parent.nodes[n];
    } else {
      if (in.q4_parent!=0 || !in.t3->prepared()) return failure;
      const auto& parent=in.t3->parent();
      if (!q4_bounds::Certify(.5*in.t3->density().value,in.t3->area_enclosure(),&out.area)) return failure;
      out.arity=3; out.family=NodalWallParentFamily::T3Native;
      out.parent_element_id=parent.parent_element_id; out.parent_face_id=parent.parent_face_id; out.feature_id=parent.feature_id;
      for (unsigned n=0;n<3;++n) out.nodes[n]=parent.nodes[n];
    }
    if (!nodal_wall_detail::Certificate(out.area,true)) return failure;
    for (unsigned n=0;n<out.arity;++n) {
      if (out.nodes[n]>=global_nodes) return failure;
      for (unsigned j=0;j<n;++j) if (out.nodes[j]==out.nodes[n]) return failure;
    }
    Q4IntegralInterval share;
    if (!q4_bounds::DividePositive({out.area.lower,out.area.upper},out.arity,&share) ||
        !q4_bounds::Certify(out.area.value/out.arity,share,&out.share) ||
        !nodal_wall_detail::Certificate(out.share,true)) return failure;
  *output=out;
  return Report(NodalWallStatus::Ok,Status::kOk);
}

NodalWallReport BuildIndexedWeights(const NodalWallParentWeight* parents,std::uint32_t count,
    NodalWallNodeWeight* nodes,std::uint32_t global_nodes,std::uint32_t& node_count,Q4CertifiedIntegral& total) {
  WeightFeatureIndex features;
  features.Prepare(count,[&](std::size_t p) {return parents[p].feature_id;});
  for(std::uint32_t p=0;p<count;++p) {
    const auto& parent=parents[p];
    const bool duplicate_face=p&&parent.parent_element_id==parents[p-1].parent_element_id&&
      parent.parent_face_id==parents[p-1].parent_face_id;
    if(duplicate_face||features.First(parent.feature_id)<p) {
      auto report=Report(NodalWallStatus::DuplicateParent);report.parent=p;return report;
    }
    if(!AccumulateWeight(total,parent.area))return Report(NodalWallStatus::NonFiniteArithmetic);
  }
  WeightNodeFlags flags;flags.Resize(global_nodes);
  for(std::uint32_t p=0;p<count;++p)for(unsigned local=0;local<parents[p].arity;++local) {
    const auto node=parents[p].nodes[local];
    if(flags[node]==2)continue;
    flags[node]=1;
    if(!AccumulateWeight(nodes[node].area,parents[p].share))flags[node]=2;
  }
  for(std::uint32_t node=0;node<global_nodes;++node) {
    // Delay arithmetic failure reporting to preserve the old ascending-node
    // first-failure order even when another node failed at an earlier parent.
    if(flags[node]==2)return Report(NodalWallStatus::NonFiniteArithmetic,Status::kNonFiniteResult,node);
    if(flags[node]) {
      if(!Certificate(nodes[node].area,true))return Report(NodalWallStatus::NonFiniteArithmetic);
      const NodalWallNodeWeight weight{node,nodes[node].area};
      nodes[node_count++]=weight; // Compact only into consumed slots.
    }
  }
  return Report(NodalWallStatus::Ok,Status::kOk);
}
} // namespace tlfea::contact::nodal_wall_detail
