// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <array>
namespace qeph_projection_test {
using V2=std::array<double,2>;
using V3=std::array<double,3>;
struct Controls {
  int npt=3,idril=0,ifini=0,iresp=0,impl_s=0,ikproj=0;
  double tolerance=1e-8;
};
struct Geometry {
  double area=0,area_i=0,x13=0,x24=0,y13=0,y24=0,mx13=0,my13=0;
  double z1=0,ll=0,l13=0,l24=0; // LL is the CZCORP5 squared-length operand.
  std::array<V2,4> corel{};
  std::array<double,9> vq{}; // Row-major: frame columns are local basis vectors.
};
struct RateInput {
  Controls controls;
  Geometry geometry;
  V3 v13{},v24{},vhi{};
  std::array<V2,4> rlxyz{};
  std::array<V3,4> world_omega{};
};
struct Projection {
  bool planar=true,warped_defined=false;
  double z1=0;
  std::array<double,6> di{};
  std::array<V3,4> db{},vqn{};
  // On planar rows these three arrays are API-zero unused cache storage,
  // not observed native numerical values. Never compare/consume as defined.
};
struct RateResult {
  Projection projection;
  V3 v13{},v24{},vhi{};
  std::array<V2,4> rlxyz{};
};
struct ForceInput {
  Controls controls;
  Geometry geometry;
  Projection projection;
  std::array<V3,4> vf{}; // Original symmetric/antisymmetric native slots.
  std::array<V2,4> vm{};
};
struct ForceResult {std::array<V3,4> force{},couple{};};
struct CapturedRow {
  int cycle=0,original_row=0;
  double time=0,dt1=0,dt12=0;
  int ismstr=0,nlay=0,irep=0,ixfem=0;
  std::array<int,4> native_node_indices{};
  std::array<V3,4> position{},velocity{},omega{};
  RateInput rate_entry;
  RateResult rate_expected;
  ForceInput force_entry;
  ForceResult force_expected;
};
const std::array<CapturedRow,12>& CapturedRows();
} // namespace qeph_projection_test
