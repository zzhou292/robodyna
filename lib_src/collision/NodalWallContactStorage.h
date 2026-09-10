#pragma once
#include "NodalWallContactDevice.h"
#include "PreparedPlanarWallQuery.h"
#include "NodalWallContactPoint.h"
#include "NodalWallContactReduction.h"

namespace tlfea::contact::nodal_wall_device_detail {
constexpr unsigned Workers=64;
static_assert(Workers>0 && Workers<=1024,
              "One bounded worker block strides over every compact node and parent");
static_assert(MaxNodalWallDeviceNodes<=MaxNodalWallNodes && MaxNodalWallDeviceParents<=MaxNodalWallParents,
              "Device collection remains within the qualified host law capacities");
using Code=NodalWallDeviceStatus;
struct Model {
  NodalWallDeviceConfig config;
  PreparedPlanarWallQuery query;
  PlanarWallBoxCoverage coverage;
  NodalWallParentWeight* parents=nullptr;
  NodalWallNodeWeight* nodes=nullptr;
  // Compact-node incidence in original parent/local order; slots index shares.
  std::uint32_t* incident_offsets=nullptr;
  std::uint32_t* incident_slots=nullptr;
  Vec3* initial_position=nullptr;
  double* inverse_mass=nullptr;
  double rate=0;
  std::uint8_t* fixed=nullptr;
  std::uint64_t face_ids[MaxPlanarWallTriangles]{};
  std::uint32_t node_count=0,parent_count=0;
  bool prepared=false;
};
struct Control {
  Code status=Code::Ok;
  std::uint32_t node=UINT32_MAX,parent=UINT32_MAX;
  NodalWallReport point;
};
struct ActiveResults {
  NodalWallDiagnostics diagnostics;
  NodalWallParentResult* parents=nullptr;
  NodalWallPointResult* nodes=nullptr;
  std::uint64_t* wall_face=nullptr;
};
struct Storage {
  Model model;
  Control control;
  Control* node_status=nullptr;
  NodalWallPointResult* shares=nullptr;
  ActiveResults base,result;
  // Six full global-indexed arrays let the existing generic scatter preflight
  // every unique incident node privately, before one final owner publication.
  double* staged_force=nullptr;
  double* addition_error=nullptr;
};
static_assert(sizeof(NodalWallDeviceResults)==79248,"Preserve the legacy fixed-result ABI");
class PreparedModel;
NodalWallDeviceReport PrepareModel(const NodalWallDeviceConfig&,PlanarWallView,const NodalWallWeights&,
    VectorView,const double*,const std::uint8_t*,PlanarWallBox,PreparedModel*);
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
