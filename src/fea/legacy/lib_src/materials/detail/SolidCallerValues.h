// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SRHO3, MQVISCB and no-EOS MULAW/MMAIN, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "lib_src/math/Quaternion.h"

#if defined(__CUDACC__)
#define TL_SOLID_CALLER_HD __host__ __device__
#else
#define TL_SOLID_CALLER_HD
#endif

namespace tl::material::solid_caller {
struct DensityValues {
  double density_kg_m3 = 0;
  double volume_increment_m3 = 0;
};
// Inputs have already passed the owning caller's positive/finite checks.
TL_SOLID_CALLER_HD inline DensityValues LagrangianDensity(double reference_density,
    double accepted_density, double storage_volume, double current_volume) noexcept {
  DensityValues result;
  result.volume_increment_m3 = current_volume-(reference_density/accepted_density)*storage_volume;
  result.density_kg_m3 = reference_density*(storage_volume/current_volume);
  return result;
}

struct ViscosityValues {
  double pressure_pa = 0;
  double unscaled_dt_s = 0;
  double stiffness_n_m = 0;
};
// Closed no-material-viscosity profile: GEO14=1.1, GEO15=.05, GEO16/17=0,
// VD2=0, active Lagrangian point. Sound speed is the material's returned SSP.
TL_SOLID_CALLER_HD inline ViscosityValues BulkViscosity(const double (&rate)[6],
    double density, double reference_density, double volume, double length,
    double sound, double density_length_floor, double sound_speed_floor) noexcept {
  const double divergence = -rate[0]-rate[1]-rate[2];
  const double compression = ::fmax(0.0,divergence);
  const double edge = ::pow(volume,1.0/3.0);
  const double cx = sound+::sqrt(0.0);
  const double qa = 1.0*1.1;
  const double qb = 1.0*.05;
  const double qaa0 = qa*qa;
  const double qaa = qaa0*compression;
  const double qx = qb*sound+edge*qaa+
      1.0*2*0.0/::fmax(density_length_floor,density*length)+
      (0.0+1.0*0.0)/::fmax(density_length_floor,reference_density*length);
  ViscosityValues result;
  result.pressure_pa = density*compression*edge*(qaa*edge+qb*sound);
  const double equivalent_sound = ::fmax(sound_speed_floor,qx+::sqrt(qx*qx+cx*cx));
  result.unscaled_dt_s = length/equivalent_sound;
  const double inverse_dt = 1.0/result.unscaled_dt_s;
  const double rho_dt = density*inverse_dt;
  const double volume_dt = volume*inverse_dt;
  result.stiffness_n_m = rho_dt*volume_dt;
  return result;
}

struct WorkValues {
  double increment_j = 0;
  double energy_density_j_m3 = 0;
};
// EINT here is the stored density after the element's volume correction and
// before SRHO3 multiplies by storage volume. No plastic-work model is implied.
TL_SOLID_CALLER_HD inline WorkValues NoEosInternalWork(const double (&old)[6],
    const double (&now)[6], const double (&rate)[6], double dt,
    double average_volume, double volume_increment, double old_q, double new_q,
    double accepted_energy_density, double storage_volume, double volume_floor) noexcept {
  const double pressure_sum = -(old[0]+now[0]+old[1]+now[1]+old[2]+now[2])*(1.0/3.0);
  double work[6]{};
  for (unsigned i = 0; i < 3; ++i) {
    work[i] = rate[i]*(old[i]+now[i]+pressure_sum+2*0.0);
  }
  for (unsigned i = 3; i < 6; ++i) work[i] = rate[i]*(old[i]+now[i]+2*0.0);
  WorkValues result;
  result.increment_j = (average_volume*dt*
      (work[0]+work[1]+work[2]+work[3]+work[4]+work[5]+0.0)-
      volume_increment*(new_q+old_q+pressure_sum))*.5;
  const double energy = accepted_energy_density*storage_volume+result.increment_j;
  result.energy_density_j_m3 = energy/::fmax(storage_volume,volume_floor);
  return result;
}
}  // namespace tl::material::solid_caller
