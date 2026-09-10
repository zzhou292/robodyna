#include "NodalWallWeightStartup.h"

namespace tlfea::contact::nodal_wall_detail {
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
