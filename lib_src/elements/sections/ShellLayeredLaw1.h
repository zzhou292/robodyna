// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected native NIP3 SIGEPS01C/MULAWC, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "ShellNip3.h"
#include "lib_src/materials/ShellElasticLaw1Point.h"
#if defined(__CUDACC__)
#define TL_LAYERED_LAW1_HD __host__ __device__
#else
#define TL_LAYERED_LAW1_HD
#endif
namespace tl::fea::sections {
struct ShellLayeredLaw1History { tl::material::ShellElasticLaw1PointHistory point[3]{}; };
struct ShellLayeredLaw1Input {
  double strain_curvature_increment[8]{}; // XX,YY,XY,YZ,ZX,KXX,KYY,KXY.
  double reference_thickness=0,reported_thickness=0,transverse_shear_modulus=0;
};
struct ShellLayeredLaw1Result {
  ShellLayeredLaw1History history;
  double material_stress[5]{},bending_stress[3]{};
  double reported_thickness=0;
};
// Accepted force thickness remains fixed across this interval's point loop;
// reported thickness accumulates each actual SIGEPS01C contribution in order.
TL_LAYERED_LAW1_HD inline bool UpdateShellLayeredLaw1(const tl::material::ShellElasticLaw1PointParameters& p,
    const ShellLayeredLaw1History& base,const ShellLayeredLaw1Input& in,
    ShellLayeredLaw1Result& output) noexcept {
  if(!tl::material::ValidShellElasticLaw1Point(p)||!tl::math::Finite(in.reference_thickness)||
     !(in.reference_thickness>0)||!tl::math::Finite(in.reported_thickness)||
     !(in.reported_thickness>=1.e-30)||!tl::math::Finite(in.transverse_shear_modulus)||
     !(in.transverse_shear_modulus>0)) return false;
  for(double x:in.strain_curvature_increment) if(!tl::math::Finite(x)) return false;
  ShellLayeredLaw1Result next;next.reported_thickness=in.reported_thickness;
  for(unsigned layer=0;layer<3;++layer) {
    tl::material::ShellElasticLaw1PointInput input;
    Nip3LayerIncrement(in,layer,input.strain_increment);
    input.transverse_shear_modulus=in.transverse_shear_modulus;
    input.layer_thickness=LayerForceWeight(layer)*in.reference_thickness;
    input.reported_thickness=next.reported_thickness;
    tl::material::ShellElasticLaw1PointResult point;
    if(!tl::material::UpdateShellElasticLaw1Point(p,base.point[layer],input,point)) return false;
    next.history.point[layer]=point.history;next.reported_thickness=point.reported_thickness;
  }
  Nip3Resultants(next.history.point,next.material_stress,next.bending_stress);
  for(double x:next.material_stress) if(!tl::math::Finite(x)) return false;
  for(double x:next.bending_stress) if(!tl::math::Finite(x)) return false;
  output=next;return true;
}
} // namespace tl::fea::sections
#undef TL_LAYERED_LAW1_HD
