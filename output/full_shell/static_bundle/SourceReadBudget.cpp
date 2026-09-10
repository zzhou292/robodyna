#include "InputChecks.h"
namespace crash::output::full_shell::source::detail {
std::size_t SourceReadBudget(const SourceInputs& in, SourceLimits limits) {
    Require(limits.file_bytes && limits.file_bytes <= kArtifactFileCap &&
        limits.source_member_bytes && limits.source_member_bytes <= 64 * 1024 * 1024 &&
        limits.canonical_array_bytes && limits.canonical_array_bytes <= 64 * 1024 * 1024 &&
        limits.host_bytes && limits.host_bytes <= 512 * 1024 * 1024 &&
        limits.nodes && limits.nodes <= 1048576 && limits.parents && limits.parents <= 1048576,
        "Invalid static source resource limits");
    CheckUnits(in.units);
    Require(in.tire_policy == "retain_all" || in.tire_policy == "omit_original_tire_shells",
        "Undeclared source tire policy");
    Require(in.canonical_manifest.bytes && in.canonical_manifest.bytes <= limits.file_bytes &&
        in.scope_report.bytes && in.scope_report.bytes <= limits.file_bytes &&
        in.source_member.bytes && in.source_member.bytes <= limits.source_member_bytes,
        "Source input byte declarations exceed capacity");
    std::size_t budget = 0;
    AddBytes(budget, 4 * in.canonical_manifest.bytes, limits.host_bytes);
    AddBytes(budget, 4 * in.scope_report.bytes, limits.host_bytes);
    AddBytes(budget, 2 * in.source_member.bytes, limits.host_bytes);
    return budget;
}
} // namespace crash::output::full_shell::source::detail
