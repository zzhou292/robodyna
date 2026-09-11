// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SRCOOR3/SGCOOR3/SDEFO3/SDLEN3, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid24ForceDerivatives.h"
#include "lib_src/elements/solid_common/FrameTensor.h"
#include "lib_src/elements/solid_common/SolidCharacteristicLength.h"

namespace tl::fea::solid24::force_detail {
TL_BRICK_HD inline ForceStatus CurrentKinematics(const Reference& reference,
    const PrescribedInterval& interval,ForceGeometry& g) noexcept {
  Vec3 world[8];
  for (unsigned n=0; n<8; ++n) world[n]=interval.position_m[reference.source_slot(n)];
  if (!brick::CyclicFrame(world,g.current.frame)) return ForceStatus::InvalidGeometry;
  for (unsigned n=0; n<8; ++n) {
    g.current.local_position_m[n]=brick::Local(g.current.frame,world[n]);
    g.local_velocity_m_s[n]=brick::Local(g.current.frame,interval.velocity_m_s[reference.source_slot(n)]);
    if (!brick::Finite(g.current.local_position_m[n]) || !brick::Finite(g.local_velocity_m_s[n]))
      return ForceStatus::NonfiniteResult;
  }
  ForceStatus status=CurrentDerivatives(g);
  if (status!=ForceStatus::Success) return status;
  if (!brick::Law42CharacteristicLength(g.current.local_position_m,g.current.volume_m3,
      g.current.characteristic_length_m)) return ForceStatus::InvalidGeometry;
  double rate[9];
  for (unsigned row=0; row<3; ++row) for (unsigned col=0; col<3; ++col)
    rate[3*row+col]=Gradient(g.derivative_per_m[col],g.local_velocity_m_s,row);
  const double xx=rate[0],xy=rate[1],xz=rate[2],yx=rate[3],yy=rate[4];
  const double yz=rate[5],zx=rate[6],zy=rate[7],zz=rate[8],h=.5*interval.dt_s;
  auto& d=g.engineering_rate_per_s;
  d[0]=xx-h*(xx*xx+yx*yx+zx*zx);
  d[1]=yy-h*(yy*yy+zy*zy+xy*xy);
  d[2]=zz-h*(zz*zz+xz*xz+yz*yz);
  double a=h*(xx*xy+yx*yy+zx*zy);
  d[3]=(xy-a)+(yx-a);
  a=h*(yy*yz+zy*zz+xy*xz);
  d[4]=(yz-a)+(zy-a);
  a=h*(zz*zx+xz*xx+yz*yx);
  d[5]=(xz-a)+(zx-a);
  for (double value:d) if (!tl::math::Finite(value)) return ForceStatus::NonfiniteResult;
  Vec3 displacement[8]{};
  const auto& source=reference.input();
  const auto& last=source.position_m[reference.source_slot(7)];
  for (unsigned n=0; n<7; ++n) {
    const auto& old=source.position_m[reference.source_slot(n)];
    displacement[n]={world[n].x-world[7].x-(old.x-last.x),
                     world[n].y-world[7].y-(old.y-last.y),
                     world[n].z-world[7].z-(old.z-last.z)};
  }
  double p[3][4],gradient[9];
  ReferenceDerivatives(*reference.reference_jacobian(),p);
  for (unsigned row=0; row<3; ++row) for (unsigned col=0; col<3; ++col)
    gradient[3*row+col]=Gradient(p[col],displacement,row);
  // Shared exact SORDEFT3 implementation is supplied by the S6Z lane.
  if (!brick::MaterialGradient(g.current.frame,gradient,g.material_displacement_gradient))
    return ForceStatus::NonfiniteResult;
  return ForceStatus::Success;
}
} // namespace tl::fea::solid24::force_detail
