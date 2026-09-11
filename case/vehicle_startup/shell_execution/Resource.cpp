#include "Internal.h"
#include "lib_utils/BoundedArena.h"

namespace crash::cases::vehicle_startup::shell_execution {
namespace detail {
void CheckLimits(const Limits& limits) {
    const Limits hard;
    const std::size_t requested[]{limits.host_bytes, limits.catalog.max_parents, limits.catalog.max_nodes,
        limits.catalog.max_definitions, limits.catalog.max_owned_bytes, limits.catalog.max_startup_scratch_bytes,
        limits.failure.max_parents, limits.failure.max_host_bytes, limits.execution.max_parents,
        limits.execution.max_nodes, limits.execution.max_host_bytes, limits.physical.max_parents,
        limits.physical.max_nodes, limits.physical.max_host_bytes};
    const std::size_t maximum[]{hard.host_bytes, hard.catalog.max_parents, hard.catalog.max_nodes,
        hard.catalog.max_definitions, hard.catalog.max_owned_bytes, hard.catalog.max_startup_scratch_bytes,
        hard.failure.max_parents, hard.failure.max_host_bytes, hard.execution.max_parents,
        hard.execution.max_nodes, hard.execution.max_host_bytes, hard.physical.max_parents,
        hard.physical.max_nodes, hard.physical.max_host_bytes};
    for (unsigned i = 0; i < std::size(requested); ++i)
        Require(requested[i] && requested[i] <= maximum[i], "Invalid complete shell execution limit");
    // Failure device budgets are unused by immutable host composition.
    Require(limits.failure.max_device_bytes == hard.failure.max_device_bytes,
            "Host execution does not select a failure device arena");
}
Forecast ForecastPayload(std::size_t source_bytes, std::size_t fixed_bytes, std::size_t parts,
    std::size_t parents, std::size_t curves, const Limits& limits) {
    CheckLimits(limits);
    Require(parts && parents && parts <= limits.catalog.max_definitions && curves <= parts &&
        parents <= limits.catalog.max_parents && parents <= limits.failure.max_parents &&
        parents <= limits.execution.max_parents && parents <= limits.physical.max_parents,
        "Complete shell execution packing exceeds entity bounds");
    tl::util::BoundedArenaLayout packing(limits.host_bytes), native(limits.host_bytes), total(limits.host_bytes);
    tl::util::ArenaRegion ignored;
    Require(packing.Append<fe::ShellPlasticityCurveInput>(curves, ignored) &&
        packing.Append<fe::ShellPlasticityMaterialInput>(parts, ignored) &&
        packing.Append<fe::ShellPlasticitySectionInput>(parts, ignored) &&
        packing.Append<fe::ShellPlasticityParentInput>(parents, ignored) &&
        packing.Append<fe::ShellFailureParentInput>(parents, ignored), "Shell execution packing overflows host cap");
    for (std::size_t bytes : {limits.catalog.max_owned_bytes, limits.catalog.max_startup_scratch_bytes,
            limits.failure.max_host_bytes, limits.execution.max_host_bytes, limits.physical.max_host_bytes})
        Require(native.Append<unsigned char>(bytes, ignored), "Shell execution native reservations exceed host cap");
    Require(total.Append<unsigned char>(source_bytes, ignored) && total.Append<unsigned char>(fixed_bytes, ignored) &&
        total.Append<unsigned char>(packing.bytes(), ignored) && total.Append<unsigned char>(native.bytes(), ignored),
        "Inclusive shell execution source/startup exceeds host cap");
    return {source_bytes, fixed_bytes, packing.bytes(), native.bytes(), total.bytes()};
}
} // namespace detail
} // namespace crash::cases::vehicle_startup::shell_execution
