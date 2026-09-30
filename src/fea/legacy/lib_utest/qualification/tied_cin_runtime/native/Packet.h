#pragma once
// Qualification-only serial, 3D, IRODDL1/WEIGHT1/IDEL2=1 packet.
// Node fields are xyz-interleaved; topology uses one-based native indices.
// Scalars are by value, all arrays have the extents stated below. A nonzero
// status preserves every mutable caller array. No runtime state is owned here.
extern "C" void tl_cin_native_stage(int nodes, int rows, const int* masters4,
    const int* secondary, const double* position3, const double* st2,
    double* force3, double* couple3, double* mass, double* inertia,
    double* stiffness, double* rotational_stiffness, double* saved_mass,
    double* saved_inertia, double* numerical_mass, double* force_integral6,
    double* dpara7, double* velocity3, double* omega3, double* acceleration3,
    double* angular_acceleration3, double time, double step, double kick, int* status);

// Exact INIEND selected seed: output arrays have one value per NSV row.
extern "C" void tl_cin_native_seed(int nodes, int rows, const int* secondary,
    const double* mass, const double* inertia, double* saved_mass,
    double* saved_inertia, int* status);

// Complete CHK2MSR3NB serial caller. Shells precede triangles in TAGEL;
// connectivity is native one-based. ITAG2=1 selects the actual active-element
// containment check even when every patch node also has deleted incidence.
// Outputs are signed NSV plus the current (possibly restored) secondary M/J.
extern "C" void tl_cin_native_witness(int nodes, int rows, int quads, int triangles,
    const int* masters4, const int* secondary, const int* quads4,
    const int* triangles3, const int* active, const int* node_active,
    const double* saved_mass, const double* saved_inertia, int* signed_secondary,
    double* mass, double* inertia, int* status);
