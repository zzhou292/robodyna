#pragma once

// Qualification-only binding for pinned native CHVIS3. The original arithmetic
// and COMMON layouts are compiled unchanged. No public production solver API.
// Fixed domain: 1..16 active planar quads, ISMSTR1/2, IHBE1, NPT3, no STI branch.
// All per-element arrays are component-major: array[component*n + element].
// Fields are the exact local CHVIS3 geometry/material inputs; callers validate
// physical mesh geometry independently. Local V/W vectors have 12 components:
// node0 xyz, node1 xyz, node2 xyz, node3 xyz. HOUR has 5 components. The last two
// HOUR slots are instantaneous rotational moments, not accumulated histories.
// Controls = HVISC, HVLIN, HELAS; these three values are shared by the batch.
// force/couple are restoring vectors: negative native H/B. Work is the native
// per-element increment (positive dissipation for fresh viscous-only states).
// Output arrays must have the stated capacity and not overlap each other.
// Inputs are borrowed/read only; output publication happens only on success.
// Calls are serialized internally because the exact native source uses COMMON.
enum CrashChvis3Field {
  CHVIS_PX1, CHVIS_PX2, CHVIS_PY1, CHVIS_PY2, CHVIS_AREA,
  CHVIS_VHX, CHVIS_VHY, CHVIS_THICKNESS, CHVIS_YOUNG, CHVIS_NU,
  CHVIS_RHO, CHVIS_SSP, CHVIS_SHF, CHVIS_H1, CHVIS_H2, CHVIS_H3,
  CHVIS_SRH1, CHVIS_SRH2, CHVIS_SRH3, CHVIS_DT, CHVIS_FIELD_COUNT
};

#ifdef __cplusplus
extern "C" {
#endif
// Return: 0 success, 1 invalid input, 2 nonfinite/inconsistent native output.
int crash_chvis3_native(int n, int ismstr, const double* controls,
                       const double* fields, const double* local_v,
                       const double* local_w, const double* hour_in,
                       double* hour_out, double* restoring_force,
                       double* restoring_couple, double* work_increment);
#ifdef __cplusplus
}
#endif
