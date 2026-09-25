// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "../geometry/Prepare.h"
#include "../geometry/HistoryChecks.h"
namespace tlfea::contact::radioss_type25::selection::detail {
namespace v = tl::math::fixed3;
namespace g = geometry_detail;
inline constexpr double onep03 = 1. + 3. / 100.;
inline constexpr double zep01 = 1. / 100.;
TL_MATH_HOST_DEVICE inline bool Supported(const Profile& p) {
  return p.gap_mode == 1 && p.initial_penetration == 5 && p.local_processor == 1 &&
      !p.foreign_rows && !p.thermal && !p.gap_loading;
}
TL_MATH_HOST_DEVICE inline bool Valid(const NativePairInput& in, const NativeGeometryHistory& prior) {
  if (!in.key.secondary_source_id || in.key.history_index == SIZE_MAX || in.key.main_segment <= 0 ||
      in.local_main <= 0 || in.occurrence == SIZE_MAX ||
      in.key.secondary_source_id != prior.secondary_source_id || in.key.generation != prior.generation ||
      !g::ValidRow(prior.row) || !v::Finite(in.secondary) ||
      !tl::math::Finite(in.main_coefficient) || !tl::math::Finite(in.secondary_coefficient) ||
      !normal_detail::Nonnegative(in.secondary_gap) || !normal_detail::Nonnegative(in.main_gap_max) ||
      !normal_detail::Nonnegative(in.radiation_range) || !tl::math::Finite(in.applied_gap)) return false;
  for (unsigned i = 0; i < 4; ++i) {
    if (!in.main_node_ids[i] || !v::Finite(in.main_vertices[i]) ||
        !v::Finite(g::Promote(in.normal_slot[i])) || !normal_detail::Nonnegative(in.main_gap[i])) return false;
    for (unsigned j = 0; j < 2; ++j) if (!v::Finite(g::Promote(in.vertex_bisector[i][j]))) return false;
    for (unsigned j = 0; j < i; ++j) {
      if (in.main_node_ids[i] == in.main_node_ids[j] &&
          (!tl::math::SameScalarBits(in.main_vertices[i].x, in.main_vertices[j].x) ||
           !tl::math::SameScalarBits(in.main_vertices[i].y, in.main_vertices[j].y) ||
           !tl::math::SameScalarBits(in.main_vertices[i].z, in.main_vertices[j].z))) return false;
      if (in.boundary_ids[i] && in.boundary_ids[i] == in.boundary_ids[j])
        for (unsigned k = 0; k < 2; ++k) {
          const auto a=in.vertex_bisector[i][k],b=in.vertex_bisector[j][k];
          if (!tl::math::SameScalarBits(a.x,b.x)||!tl::math::SameScalarBits(a.y,b.y)||!tl::math::SameScalarBits(a.z,b.z)) return false;
        }
    }
  }
  return true;
}
struct Work {
  g::Work frame;
  Vector relative[4]{}, from_secondary{}, normal[4]{};
  double hlb[4]{}, hlc[4]{}, along[4]{}, plane_distance[4]{};
  double center_gap = 0;
  bool shell = false;
};
TL_MATH_HOST_DEVICE inline void Prepare(const NativePairInput& in, Work& w) {
  const bool triangle=in.main_node_ids[2]==in.main_node_ids[3];
  g::PrepareMainFrame(triangle,in.main_vertices,in.normal_slot,w.frame);
  w.shell=in.segment_type!=0||in.secondary_gap>0;
  w.from_secondary=v::Subtract(w.frame.point[4],in.secondary);
  w.center_gap=triangle?in.main_gap[2]:
      .25*(in.main_gap[0]+in.main_gap[1]+in.main_gap[2]+in.main_gap[3]);
  for(unsigned i=0;i<4;++i)w.relative[i]=v::Subtract(w.frame.point[i],in.secondary);
}
TL_MATH_HOST_DEVICE inline void Initialize(const NativePairInput& in,
    const NativeGeometryHistory& prior, NativeRetainedResult& result) {
  result.history=prior;result.cache.key=in.key;result.cache.occurrence=in.occurrence;
  result.cache.local_main=in.local_main;result.distance_squared=native_constant::ep20;
  result.classification_product=in.main_coefficient*::fabs(in.secondary_coefficient);
  result.active=result.classification_product>0;
  int old=prior.row.irtlm[1]%5;if(old<0)old=-old;
  result.prior_subtriangle=old;
  for(auto& s:result.sector) {
    s.penetration=native_constant::ep20;s.distance_squared=native_constant::ep20;
    s.defined=FarDefined|PenetrationDefined|DistanceSquaredDefined;
  }
}
} // namespace tlfea::contact::radioss_type25::selection::detail
