#pragma once
#include "output/physical_run/ViewerInput.h"
#include <cstddef>
#include <cstdint>
#include <string>

namespace crash::cases::vehicle_run::test::retained {
inline constexpr double StepS = 2e-7;
// The live gate supplies values from its actual source forecast/controller.
// The retained-archive gate supplies its explicit V5 contract and pinned summary.
// These are qualification expectations, never solver or publication authority.
struct Expected {
    std::uint64_t source_id = 0;
    std::size_t selected_parents = 0, event_capacity = 0;
    std::uint64_t last_event_count = 0, last_policy_digest = 0;
    double last_potential_j = 0;
    std::string stop_reason;
};
void CheckTwoIntervalArchive(const std::filesystem::path&,
    const output::full_shell::RecordFile& viewer_input,
    const output::full_shell::RecordFile& summary_file, const Expected&);
} // namespace crash::cases::vehicle_run::test::retained
