#include "NodalWallContactIncidence.h"
#include <algorithm>

namespace tlfea::contact::nodal_wall_device_detail {
NodalWallDeviceReport BuildVehicleIncidence(Model& model) {
  const auto globals=model.config.owner.node_count;
  tl::util::BoundedStartupArray<std::uint32_t,0> compact;
  compact.Resize(globals);
  std::fill_n(compact.data(),globals,UINT32_MAX);
  std::fill_n(model.incident_offsets,model.node_count+1,0);
  for(unsigned i=0;i<model.node_count;++i) {
    const auto node=model.nodes[i].node;
    if(node>=globals||compact[node]!=UINT32_MAX)
      return {Code::InvalidInput,"Contact compact-node identity is invalid",node};
    compact[node]=i;
  }
  for(unsigned p=0;p<model.parent_count;++p) {
    const auto& parent=model.parents[p];
    for(unsigned l=0;l<parent.arity;++l) {
      const auto node=parent.nodes[l];
      if(node>=globals||compact[node]==UINT32_MAX)
        return {Code::InvalidInput,"Contact compact nodes do not cover every native parent share",node,p};
      ++model.incident_offsets[compact[node]+1];
    }
  }
  for(unsigned i=1;i<=model.node_count;++i)
    model.incident_offsets[i]+=model.incident_offsets[i-1];
  // Reuse starts as cursors while traversing exactly original parent/local
  // order. Every compact slice therefore has the legacy directed sum order.
  for(unsigned p=0;p<model.parent_count;++p)
    for(unsigned l=0;l<model.parents[p].arity;++l) {
      const auto i=compact[model.parents[p].nodes[l]];
      model.incident_slots[model.incident_offsets[i]++]=4*p+l;
    }
  // Cursors now contain ends. Restore the original immutable offset table.
  for(unsigned i=model.node_count;i>0;--i)
    model.incident_offsets[i]=model.incident_offsets[i-1];
  model.incident_offsets[0]=0;
  return {Code::Ok,"Source-ordered contact incidence prepared"};
}
} // namespace tlfea::contact::nodal_wall_device_detail
