#pragma once
#include "NodalWallContactDevice.h"
#include "PreparedPlanarWallQuery.h"
#include "NodalWallContactPoint.h"
#include "NodalWallContactReduction.h"

namespace tlfea::contact::nodal_wall_device_detail {
constexpr unsigned OwnerNodes=tl::fea::MaxShellCollectionNodes,Workers=128;
static_assert(MaxNodalWallDeviceNodes==OwnerNodes && MaxNodalWallDeviceParents==128 && Workers==128,
              "One bounded worker block covers every compact node and parent");
static_assert(MaxNodalWallDeviceNodes<=MaxNodalWallNodes && MaxNodalWallDeviceParents<=MaxNodalWallParents,
              "Device collection remains within the qualified host law capacities");
using Code=NodalWallDeviceStatus;
struct Model {
  NodalWallDeviceConfig config;
  PreparedPlanarWallQuery query;
  PlanarWallBoxCoverage coverage;
  NodalWallParentWeight parents[MaxNodalWallDeviceParents]{};
  NodalWallNodeWeight nodes[MaxNodalWallDeviceNodes]{};
  Vec3 initial_position[OwnerNodes]{};
  double inverse_mass[OwnerNodes]{},rate=0;
  std::uint8_t fixed[OwnerNodes]{};
  std::uint64_t face_ids[MaxPlanarWallTriangles]{};
  std::uint32_t node_count=0,parent_count=0;
  bool prepared=false;
};
struct Control {
  Code status=Code::Ok;
  std::uint32_t node=UINT32_MAX,parent=UINT32_MAX;
  NodalWallReport point;
};
struct Storage {
  Model model;
  Control control,node_status[MaxNodalWallDeviceNodes]{};
  NodalWallPointResult shares[MaxNodalWallDeviceParents*4]{};
  NodalWallDeviceResults base,result;
  // Six full global-indexed arrays let the existing generic scatter preflight
  // every unique incident node privately, before one final owner publication.
  double staged_force[6*OwnerNodes]{},addition_error[MaxNodalWallDeviceNodes]{};
};
static_assert(sizeof(Storage)<=MaxNodalWallDeviceBytes,"Complete contact storage, one allocation");
static_assert(sizeof(Model)==105976 && sizeof(NodalWallDeviceResults)==79248 && sizeof(Storage)==471864,
              "Pinned 64-bit host/CUDA storage ledger; review an ABI change before allocating");
NodalWallDeviceReport PrepareModel(const NodalWallDeviceConfig&,PlanarWallView,const NodalWallWeights&,
    VectorView,const double*,const std::uint8_t*,PlanarWallBox,Model*);
TL_SURFACE_HD inline bool Fail(Control& c,Code code,unsigned node=UINT32_MAX,unsigned parent=UINT32_MAX) {
  c.status=code; c.node=node; c.parent=parent; return false;
}
TL_SURFACE_HD inline bool Inside(Vec3 x,const PlanarWallBox& b) {
  return IsFinite(x) && x.y>=b.minimum.y && x.y<=b.maximum.y && x.z>=b.minimum.z && x.z<=b.maximum.z;
}
TL_SURFACE_HD inline bool Radius(double value,Q4IntegralInterval truth,double* radius) {
  double a=0,b=0;
  if (!q4_bounds::Finite(truth) || !q4_bounds::AbsoluteDifferenceUpper(value,truth.lower,&a) ||
      !q4_bounds::AbsoluteDifferenceUpper(value,truth.upper,&b)) return false;
  *radius=a>b?a:b; return true;
}
TL_SURFACE_HD inline bool SignedScale(Q4IntegralInterval a,double b,Q4IntegralInterval* out) {
  if (b>=0) return q4_bounds::Scale(a,b,out);
  return q4_bounds::Scale({-a.upper,-a.lower},-b,out);
}
TL_SURFACE_HD inline bool AddUpper(double a,double b,double* out) { return mass_detail::UpperSum(a,b,out); }
} // namespace tlfea::contact::nodal_wall_device_detail

namespace tlfea::contact {
struct NodalWallContactDevice::Impl {
  ~Impl();
  nodal_wall_device_detail::Storage* device=nullptr;
  NodalWallDeviceConfig config;
  nodal_wall_device_detail::Control control;
  NodalWallDeviceResults staging;
  NodalWallDiagnostics available;
  tl::fea::NodalAssemblyView base_view;
  double rate=0;
  std::uint64_t last_epoch=0,last_attempt=0,last_candidate_attempt=0;
  cudaStream_t stream=nullptr;
  bool usable=true,has_base=false,has_results=false,has_stream=false;
  NodalWallDeviceReport Check(cudaError_t);
  NodalWallDeviceReport ReadControl(cudaStream_t);
  NodalWallDeviceReport FailAssembly(const tl::fea::NodalAssemblyView&,NodalWallDeviceReport);
};
} // namespace tlfea::contact
