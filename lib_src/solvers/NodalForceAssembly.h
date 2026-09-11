#pragma once
#include "FENodalStateView.h"
#include "../math/Fixed3.h"
#include "../math/Quaternion.h"

#if defined(__CUDACC__)
#define TL_NODAL_ASSEMBLY_HD __host__ __device__
#else
#define TL_NODAL_ASSEMBLY_HD
#endif
namespace tl::fea {
enum class NodalForceAssemblyStatus { Success, InvalidView, InvalidConnectivity, NonfiniteResult };

// Serialized additive scatter of a completed element contribution. All six
// destination ranges are disjoint and do not overlap inputs. Epoch/owner and
// lifetime are the coordinator's responsibility. Failure preserves this entire
// contribution; a coordinator discards the assembly after ANY failure.
namespace nodal_force_detail {
template<unsigned Count, bool WithRotation>
TL_NODAL_ASSEMBLY_HD inline NodalForceAssemblyStatus Accumulate(
    const std::size_t* nodes,const tl::math::Vec3* force,const tl::math::Vec3* couple,
    DeviceNodalForceView view,int sign=1) {
  static_assert(Count>0&&Count<=8,"Bounded shell, spring and solid scatter");
  constexpr unsigned Channels=WithRotation?6:3;
  if(!nodes||!force||!view.node_count||!view.force_x||!view.force_y||!view.force_z||
     (WithRotation&&(!couple||!view.couple_x||!view.couple_y||!view.couple_z))||(sign!=1&&sign!=-1))
    return NodalForceAssemblyStatus::InvalidView;
  double* arrays[6]={view.force_x,view.force_y,view.force_z,view.couple_x,view.couple_y,view.couple_z};
  for(unsigned c=0;c<Channels;++c) for(unsigned other=0;other<c;++other)
    if(arrays[c]==arrays[other]) return NodalForceAssemblyStatus::InvalidView;
  double candidate[Count][Channels];
  for(unsigned n=0;n<Count;++n) {
    if(nodes[n]>=view.node_count) return NodalForceAssemblyStatus::InvalidConnectivity;
    for(unsigned other=0;other<n;++other)
      if(nodes[n]==nodes[other]) return NodalForceAssemblyStatus::InvalidConnectivity;
    for(unsigned c=0;c<Channels;++c) {
      const auto v=c<3?force[n]:couple[n];
      const double value=c%3==0?v.x:c%3==1?v.y:v.z;
      candidate[n][c]=arrays[c][nodes[n]]+(sign==1?value:-value);
      if(!tl::math::Finite(candidate[n][c])) return NodalForceAssemblyStatus::NonfiniteResult;
    }
  }
  for(unsigned n=0;n<Count;++n) for(unsigned c=0;c<Channels;++c) arrays[c][nodes[n]]=candidate[n][c];
  return NodalForceAssemblyStatus::Success;
}
} // namespace nodal_force_detail
template<unsigned Count>
TL_NODAL_ASSEMBLY_HD inline NodalForceAssemblyStatus AccumulateNodalForces(
    const std::size_t* nodes,const tl::math::Vec3* force,const tl::math::Vec3* couple,
    DeviceNodalForceView view,int sign=1) {
  return nodal_force_detail::Accumulate<Count,true>(nodes,force,couple,view,sign);
}
// Solids have no rotational contribution. In particular, an absent/zero
// rotation does not permit rewriting another producer's couple or its sign bit.
template<unsigned Count>
TL_NODAL_ASSEMBLY_HD inline NodalForceAssemblyStatus AccumulateNodalTranslationalForces(
    const std::size_t* nodes,const tl::math::Vec3* force,
    DeviceNodalForceView view,int sign=1) {
  return nodal_force_detail::Accumulate<Count,false>(nodes,force,nullptr,view,sign);
}
} // namespace tl::fea
#undef TL_NODAL_ASSEMBLY_HD
