#pragma once
#include "NodalWallContactDevice.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

namespace tlfea::contact::nodal_wall_device_detail {
// Metadata-only preflight. No pointed-to output/identity value is read here.
inline bool ValidResultView(const NodalWallDeviceResultView& view,std::size_t parents,std::size_t nodes,
    const NodalWallDiagnostics* expected,NodalWallDeviceProfile profile=NodalWallDeviceProfile::Legacy) noexcept {
  const bool vehicle=profile==NodalWallDeviceProfile::Vehicle;
  if((profile!=NodalWallDeviceProfile::Legacy&&!vehicle)||!parents||!nodes||
      parents>(vehicle?MaxVehicleNodalWallDeviceParents:MaxActiveNodalWallDeviceParents)||
      nodes>(vehicle?MaxVehicleNodalWallDeviceNodes:MaxActiveNodalWallDeviceNodes)||
      view.parent_capacity!=parents||view.node_capacity!=nodes) return false;
  const void* ranges[]{view.diagnostics,view.parents,view.nodes,view.wall_face};
  const std::size_t bytes[]{sizeof(NodalWallDiagnostics),parents*sizeof(NodalWallParentResult),
      nodes*sizeof(NodalWallPointResult),nodes*sizeof(std::uint64_t)};
  const std::size_t alignment[]{alignof(NodalWallDiagnostics),alignof(NodalWallParentResult),
      alignof(NodalWallPointResult),alignof(std::uint64_t)};
  for(unsigned i=0;i<4;++i) {
    if(reinterpret_cast<std::uintptr_t>(ranges[i])%alignment[i] ||
       !tl::fea::trial_identity::Disjoint(ranges[i],bytes[i],expected,sizeof(*expected))||
       !tl::fea::trial_identity::Disjoint(ranges[i],bytes[i],&view,sizeof(view))) return false;
    for(unsigned j=0;j<i;++j)
      if(!tl::fea::trial_identity::Disjoint(ranges[i],bytes[i],ranges[j],bytes[j])) return false;
  }
  return true;
}
} // namespace tlfea::contact::nodal_wall_device_detail
