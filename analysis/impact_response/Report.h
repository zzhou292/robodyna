#pragma once

#include "Connectivity.h"
#include "output/physical_run/ViewerInput.h"
#include <map>

namespace crash::analysis::impact_response {

struct RunMetadata {
    output::full_shell::Identity identity;
    output::full_shell::source::Units source_units;
    output::full_shell::RecordFile viewer_input;
    output::full_shell::RecordFile archive_manifest;
    output::full_shell::RecordFile source_canonical_manifest;
    output::full_shell::RecordFile source_scope_report;
    output::full_shell::RecordFile source_member;
    output::full_shell::RecordFile run_summary;
    output::full_shell::RecordFile connectivity_report;
    std::string tire_policy;
    std::string mapping_sha256;
    std::map<std::uint64_t, std::string> source_part_titles;
    std::size_t replay_peak_host_bytes = 0;
    std::size_t mapped_nodes = 0;
    std::size_t mapped_parents = 0;
    std::size_t mapped_triangles = 0;
    std::size_t stored_native_points = 0;
    std::uint64_t planned_intervals = 0;
    std::uint64_t accepted_intervals = 0;
    bool horizon_complete = false;
    std::string stop_reason;
    double fixed_dt_s = 0;
    output::physical_run::Profile profile;
};

struct SummaryEvidence {
    std::string schema;
    std::string status;
    std::string reason;
    std::string physical_profile;
    double actual_completed_time_s = 0;
    double peak_wall_force_n = 0;
    double peak_wall_penetration_m = 0;
    double peak_same_mask_potential_j = 0;
    double last_same_mask_potential_j = 0;
    double last_removed_potential_j = 0;
    double solid_reported_plastic_work_sum_j = 0;
    double beam_reported_plastic_work_sum_j = 0;
    double translation_departure_last_m = 0;
    double translation_departure_peak_m = 0;
    double velocity_departure_last_m_s = 0;
    double velocity_departure_peak_m_s = 0;
    double orientation_component_departure_last = 0;
    double orientation_component_departure_peak = 0;
    double spin_component_last_rad_s = 0;
    double spin_component_peak_rad_s = 0;
};

RunMetadata MakeRunMetadata(const output::physical_run::Replay&,
    const output::physical_run::ViewerInput&,
    output::full_shell::RecordFile viewer_input,
    output::full_shell::RecordFile run_summary,
    output::full_shell::RecordFile connectivity_report);

SummaryEvidence VerifyRunSummary(const std::string& bytes,
    const RunMetadata&, const PlasticityResult&);

output::Document BuildReport(const RunMetadata&, const SummaryEvidence&,
    const PlasticityResult&, const ConnectivityEvidence&);

}  // namespace crash::analysis::impact_response
