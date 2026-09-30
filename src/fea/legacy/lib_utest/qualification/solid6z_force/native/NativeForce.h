// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
// Native-order geometry inputs; C++ restores the independently returned source permutation.
extern "C" void solid6z_force_native(const double* parameters4,
    const double* reference_position18, const double* reference11,
    const double* accepted21, const double* position18, const double* velocity18,
    const double* step3, double* geometry98, double* material33,
    double* history21, double* forces54, double* stabilization28, int* status);
// Generates virgin history, exact XREF positions and zero dt internally.
extern "C" void solid6z_force_initial_native(const double* parameters4,
    const double* reference_position18, const double* reference11,
    const double* uniform_velocity3, const double* stabilization_profile2,
    double* geometry98, double* material33, double* history21, double* forces54,
    double* stabilization28, int* status);
