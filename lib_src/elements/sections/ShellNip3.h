// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected native centered NIP3 integration, OpenRadioss (C) 2026 Siemens.
#pragma once
#if defined(__CUDACC__)
#define TL_NIP3_HD __host__ __device__
#else
#define TL_NIP3_HD
#endif
namespace tl::fea::sections {
// Existing LAW44 API names and exact expression/reduction order are retained.
TL_NIP3_HD inline double LayerPosition(unsigned i) noexcept { return .5*(static_cast<double>(i)-1.); }
TL_NIP3_HD inline double LayerForceWeight(unsigned i) noexcept { return i==1?.5:.25; }
TL_NIP3_HD inline double LayerMomentWeight(unsigned i) noexcept {
  return (static_cast<double>(i)-1.)*static_cast<double>(0.0833333f);
}
template<class Input>
TL_NIP3_HD inline void Nip3LayerIncrement(const Input& in,unsigned layer,double (&dx)[5]) noexcept {
  const double z=LayerPosition(layer)*in.reference_thickness;
  for(unsigned c=0;c<3;++c)
    dx[c]=in.strain_curvature_increment[c]+z*in.strain_curvature_increment[c+5];
  for(unsigned c=3;c<5;++c) dx[c]=in.strain_curvature_increment[c];
}
// FOR and MOM are native stress-like quantities; physical resultants are FOR*t
// and MOM*t*t. Separate native moment weights must not become WF*Z or exact1/12.
template<class PointHistory>
TL_NIP3_HD inline void Nip3Resultants(const PointHistory (&point)[3],
    double (&force)[5],double (&moment)[3]) noexcept {
  for(double& x:force) x=0;
  for(double& x:moment) x=0;
  for(unsigned p=0;p<3;++p) {
    for(unsigned c=0;c<5;++c) force[c]=force[c]+LayerForceWeight(p)*point[p].stress[c];
    for(unsigned c=0;c<3;++c) moment[c]=moment[c]+LayerMomentWeight(p)*point[p].stress[c];
  }
}
} // namespace tl::fea::sections
#undef TL_NIP3_HD
