// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
// Parameter order: mu, nu, rho0, cutoff. Base: stress6, rho, EINTdensity, Q.
// Step: dt, current volume, stored reference volume, characteristic length.
// Output: 9history, 13point, 6strain, 5{dV,Vavg,work,rawDT,STI}.
extern "C" void law42_solid_caller_native(const double* parameters,const double* base,
    const double* gradient,const double* rate,const double* step,double* values,int* status);
extern "C" void law42_solid_initial_modulus_native(const double* parameters,double* modulus);
