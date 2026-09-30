// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
extern "C" void type13_native_endpoint_stiffness(
    const double* kt, const double* kr, const double* ct, const double* cr,
    const double* mass, const double* inertia, const double* element_mass,
    const double* element_inertia, const double* active, double* result);
