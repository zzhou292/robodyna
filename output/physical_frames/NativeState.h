#pragma once
#include "lib_src/solvers/FENodalState.h"
namespace crash::output::physical_frames {
// Read-only physical-domain order, valid until the next successful capture or
// destruction. Initial reactions are unavailable according to stamp, not forces
// evaluated at t=0. Raw M/J are physical source coefficients, not inverses.
struct NativeAcceptedState {
    tl::fea::NodalStamp stamp;
    const double* position_xyz=nullptr;const double* velocity_xyz=nullptr;
    const double* orientation_wxyz=nullptr;const double* spin_xyz=nullptr;
    const double* reaction_force_xyz=nullptr;const double* reaction_couple_xyz=nullptr;
    const double* mass_kg=nullptr;const double* inertia_kg_m2=nullptr;
    std::size_t nodes=0;double numerical_mass_kg=0;bool available=false;
};
}
