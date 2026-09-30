// SPDX-License-Identifier: MIT
#pragma once
// Serial, test-only native bucket traversal for the qualified supplied shell
// thickness / zero secondary-shell-incidence profile. Every integer index is
// one-based. x is node-major XYZ; masters have four slots (T3 repeats slot 3).
// msr retains the original unique master-node order. Thicknesses are original
// working units, independently qualified by the source association oracle.
// No production boxes, candidate pairs or selected mappings are inputs.
// On failure all result buffers remain unchanged; status=1 input/domain,
// status=2 pair budget, status=3 nonfinite native result. Successful pairs are
// (IRECT rank, NSV rank) in the complete native enumeration order.
extern "C" void native_tied_bucket(
    int nodes,int masters,int secondaries,int master_nodes,int pair_capacity,
    const double* x,const int* irect,const int* nsv,const int* msr,
    const double* bounds_thickness,const double* projection_thickness,
    int* selected,double* st,double* distance,int* pair_count,int* pairs,
    double* bounds,int* cell_counts,int* status);
