// SPDX-License-Identifier: MIT
#pragma once
// Working-unit packet: mass/J; four K; four C. Original Ileng0 alpha=1,
// H=0, no source scaling. RBY_NODE indices address the three MS/IN slots.
extern "C" void mapped_type25_native_stiffness(const double* property,const double* length,
    const int* active,const double* mass,const double* inertia,const int* rigid_primary,
    double* stiffness);
extern "C" void mapped_type25_native_scatter(const int* node_count,const int* element_count,
    const int* endpoints,const double* stiffness,double* translation,double* rotation);
