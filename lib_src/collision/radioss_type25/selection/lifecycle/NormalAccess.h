// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "../../geometry/HistoryChecks.h"
namespace tlfea::contact::radioss_type25::selection::lifecycle::detail {
TL_MATH_HOST_DEVICE inline bool HasCurrentNormals(const Input& in) {
  const auto& v=in.current_normals;
  return v.face_normals||v.normal_count||v.references||v.reference_count;
}
// Shape/range admission only, no float reads. Full Validate checks the selected
// values in its original order. The new normal producer/owner closes readiness.
TL_MATH_HOST_DEVICE inline bool CurrentNormalRanges(const Input& in,
    geometry_detail::Range& faces,geometry_detail::Range& references) {
  const auto& v=in.current_normals;
  if(!HasCurrentNormals(in)) {faces={nullptr,0,alignof(StoredNormal)};references={nullptr,0,alignof(NormalReference)};return true;}
  if(!in.source.main_count||!in.source.normal_count||in.source.main_count>SIZE_MAX/4||
      v.normal_count!=4*in.source.main_count||v.reference_count!=in.source.normal_count||
      v.normal_count>SIZE_MAX/sizeof(StoredNormal)||v.reference_count>SIZE_MAX/sizeof(NormalReference))return false;
  const geometry_detail::Range f{v.face_normals,v.normal_count*sizeof(StoredNormal),alignof(StoredNormal)},
      r{v.references,v.reference_count*sizeof(NormalReference),alignof(NormalReference)};
  if(!geometry_detail::RangeValid(f)||!geometry_detail::RangeValid(r)||!geometry_detail::Disjoint(f,r))return false;
  faces=f;references=r;return true;
}
TL_MATH_HOST_DEVICE inline bool CurrentNormalShape(const Input& in) {
  geometry_detail::Range f{},r{};return CurrentNormalRanges(in,f,r);
}
TL_MATH_HOST_DEVICE inline const StoredNormal& FaceNormal(const Input& in,std::size_t main,unsigned slot) {
  return HasCurrentNormals(in)?in.current_normals.face_normals[4*main+slot]:in.source.mains[main].normal_slot[slot];
}
TL_MATH_HOST_DEVICE inline const NormalReference& ReferenceNormal(const Input& in,std::size_t reference) {
  return HasCurrentNormals(in)?in.current_normals.references[reference]:in.source.normals[reference];
}
template<class T> TL_MATH_HOST_DEVICE inline bool CurrentNormalsDisjoint(const Input& in,T* output,std::size_t count) {
  if(!HasCurrentNormals(in))return true; // Preserve the legacy admission path.
  if(count>SIZE_MAX/sizeof(T))return false;
  geometry_detail::Range f{},r{};
  const geometry_detail::Range out{output,count*sizeof(T),alignof(T)};
  return CurrentNormalRanges(in,f,r)&&geometry_detail::RangeValid(out)&&
      geometry_detail::Disjoint(out,f)&&geometry_detail::Disjoint(out,r);
}
} // namespace tlfea::contact::radioss_type25::selection::lifecycle::detail
