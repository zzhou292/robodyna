// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
// Qualification-only serial C/Fortran ABI. Every count/index/table is checked
// by the typed oracle adapter before native code; these are not product APIs.
extern "C" {
void rd_lifecycle_optcd(const int* counts, const int* precision_mode, const double* previous_dt,
    const double* kinematics, const int* main_nodes, const int* main_global, const int* main_role,
    const double* main_stiffness, const double* main_gap, const int* secondary_nodes,
    const double* secondary_stiffness, const double* secondary_gap, int* markers, double* metrics,
    double* friction, double* penetration, double* stiffness, int* raw_n, const int* raw_e,
    int* output_n, int* output_e, int* used, int* initial_contact, int* ordinals, int* required);
void rd_lifecycle_release_main(const int* nsn, int* markers);
void rd_lifecycle_clear_sliding(const int* nsn, int* sliding);
void rd_lifecycle_finish_markers(const int* nsn, int* markers);
void rd_lifecycle_membership(const int* counts, const int* phase,
    const int* candidates_n, const int* candidates_e, const int* main_global,
    const int* markers, int* indices, int* count);
void rd_lifecycle_prepare1(const int* counts, const int* candidates_n,
    const int* candidates_e, const int* main_nodes, const int* main_global,
    const int* normal_refs, int* markers, double* metrics, const int* cache_far,
    const double* cache_penetration, const double* cache_lb,
    const double* cache_lc, int* sliding);
void rd_lifecycle_expand(const int* counts, const int* main_nodes,
    const int* main_global, const int* main_role, const int* normal_refs,
    const int* secondary_nodes, const int* sliding, const int* markers,
    const double* main_stiffness, const int* normal_offsets,
    const int* normal_mains, const int* removal_offsets,
    const int* removed_mains, const int* removal_mode,
    int* candidates_n, int* candidates_e, int* used, int* found);
void rd_lifecycle_keep(const int* counts, int* candidates_n,
    const int* candidates_e, const int* indices, const int* main_global,
    int* markers, const double* cache_penetration, double* penetration_history,
    const int* node_ids, const int* secondary_nodes, double* friction_history,
    double* metrics, double* stiffness_history);
}
static_assert(sizeof(int) == 4, "Native oracle requires the pinned C_INT32 ABI");
