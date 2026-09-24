#include "RuntimeBudget.h"

#include "lib_utils/BoundedArena.h"
#include "output/ArtifactIO.h"

#include <algorithm>
#include <type_traits>
#include <utility>

namespace crash::cases::vehicle_self_contact::detail {
namespace {

template<class T, class = void>
struct HasSharedBackingDiscount : std::false_type {};

template<class T>
struct HasSharedBackingDiscount<T, std::void_t<
    decltype(std::declval<T>().shared_backing_discount_bytes)>>
    : std::true_type {};

std::size_t Delta(std::size_t total, std::size_t retained,
                  const char* message) {
    output::Require(total >= retained, message);
    return total - retained;
}

}  // namespace

tlfea::contact::SelfContactTransactionConfig TransactionConfig(
    RuntimeConfig config, const tl::fea::NodalStamp& owner,
    const tl::fea::ShellPhysicalPublicationIdentity& identity) noexcept {
    tlfea::contact::SelfContactTransactionConfig result;
    result.force.owner = owner;
    result.force.startup = identity.startup;
    result.force.stiffness_per_area_n_m3 =
        FirstProfileStiffnessPerAreaNPerM3;
    result.force.event_capacity = config.event_capacity;
    result.force.configuration_id = identity.configuration_id;
    result.force.qualification_id = identity.qualification_id;
    result.source_id = config.source_id;
    result.broadphase_axis = config.broadphase_axis;
    result.enable_diagnostics = config.enable_diagnostics;
    result.enable_cuda_facet_filters = config.enable_cuda_facet_filters;
    return result;
}

bool TransactionAllocationsMatch(RuntimeConfig config,
    tlfea::contact::SelfContactFacetFilterInitialization mode,
    const tlfea::contact::SelfContactTransactionForecast& requested,
    const tlfea::contact::SelfContactTransactionAllocationInfo& actual,
    const tlfea::contact::SelfContactTransactionForecast* cpu_fallback) noexcept {
    using Mode = tlfea::contact::SelfContactFacetFilterInitialization;
    const auto* expected = &requested;
    if (!config.enable_cuda_facet_filters) {
        if (mode != Mode::Disabled || cpu_fallback) return false;
    } else if (mode == Mode::Cuda) {
        if (cpu_fallback) return false;
    } else if (mode == Mode::UnsupportedHostArithmetic) {
        if (!cpu_fallback || cpu_fallback->activity.arena_bytes != requested.activity.arena_bytes ||
            cpu_fallback->device_bytes >= requested.device_bytes ||
            cpu_fallback->device_allocations >= requested.device_allocations) return false;
        expected = cpu_fallback;
    } else return false;
    return actual.activity.host_bytes == expected->activity.arena_bytes &&
        actual.device.device_bytes == expected->device_bytes &&
        actual.device.device_allocations == expected->device_allocations;
}
std::size_t EffectiveCombinedDeviceBytes(std::size_t combined,
    std::size_t requested_self, std::size_t initialized_self) {
    output::Require(requested_self <= combined && initialized_self <= requested_self,
        "Initialized self-contact allocation exceeds its composed reservation");
    return combined - requested_self + initialized_self;
}

std::size_t IncrementalSetupHost(const SetupForecast& setup) {
    output::Require(
        setup.retained_setup_reservation_bytes >=
            setup.shared_vehicle_source_reservation_bytes,
        "Self-contact setup forecast is smaller than its shared source");
    return setup.retained_setup_reservation_bytes -
        setup.shared_vehicle_source_reservation_bytes;
}

std::size_t IncrementalTransactionHost(
    const tlfea::contact::SelfContactTransactionForecast& transaction) {
    const auto regularity_source =
        transaction.regularity.retained_active_use_bytes;
    const auto broadphase_source =
        TransactionChargesBroadphaseBacking()
            ? transaction.broadphase.retained_source_bytes
            : 0;
    output::Require(
        broadphase_source <= transaction.owned_host_bytes &&
            regularity_source <=
                transaction.owned_host_bytes - broadphase_source,
        "Self-contact transaction retained-source partition is invalid");
    // Both lower modules retain shared handles into VehicleSelfContactSetup.
    // They are standalone forecast charges, not duplicate allocations here.
    return transaction.owned_host_bytes -
        broadphase_source - regularity_source;
}

bool TransactionChargesBroadphaseBacking() noexcept {
    // HEAD 493364c charges this standalone backing in transaction-owned host;
    // later compatible TL trees expose an explicit discount field and have
    // already removed it. The app remains exact against either API shape.
    return !HasSharedBackingDiscount<
        tlfea::contact::SelfContactTransactionForecast>::value;
}

std::size_t TransactionStartupScratch(
    const tlfea::contact::SelfContactTransactionForecast& transaction) {
    output::Require(
        transaction.startup_host_bytes >= transaction.owned_host_bytes,
        "Self-contact transaction startup forecast is invalid");
    const auto force_delta = Delta(
        transaction.force.startup_host_bytes,
        transaction.force.owned_host_bytes,
        "Self-contact force startup forecast is invalid");
    output::Require(
        force_delta >= transaction.force.retained_active_use_bytes,
        "Self-contact force shared startup backing is invalid");
    return std::max({
        transaction.startup_host_bytes - transaction.owned_host_bytes,
        force_delta - transaction.force.retained_active_use_bytes,
        Delta(transaction.activity.startup_host_bytes,
              transaction.activity.owned_host_bytes,
              "Self-contact activity startup forecast is invalid"),
        Delta(transaction.broadphase.startup_host_bytes,
              transaction.broadphase.owned_host_bytes,
              "Self-contact broadphase startup forecast is invalid"),
        Delta(transaction.regularity.startup_payload_bytes,
              transaction.regularity.owned_payload_bytes,
              "Self-contact regularity startup forecast is invalid")});
}

RuntimeForecast ComposeForecast(
    const vehicle_dynamics::Forecast& dynamics,
    const SetupForecast& setup,
    const tlfea::contact::SelfContactTransactionForecast& transaction,
    std::size_t fixed_bytes, RuntimeLimits limits) {
    output::Require(
        limits.host_bytes &&
            limits.host_bytes <= std::size_t{20} * 1000 * 1000 * 1000 &&
            limits.device_bytes &&
            limits.device_bytes <= (std::size_t{8} << 30),
        "Invalid complete self-contact runtime cap");
    output::Require(
        setup.shared_vehicle_source_reservation_bytes ==
            dynamics.startup.retained_source_upper_bound,
        "Self-contact setup and dynamics do not share one source reservation");
    output::Require(
        transaction.activity.owned_host_bytes &&
            transaction.activity.arena_bytes &&
            transaction.device_bytes &&
            transaction.participation.publication_host_bytes,
        "Incomplete self-contact transaction forecast");

    tl::util::BoundedArenaLayout host(limits.host_bytes);
    tl::util::BoundedArenaLayout device(limits.device_bytes);
    tl::util::ArenaRegion unused;
    for (const auto bytes : {
             dynamics.startup.retained_host_upper_bound,
             dynamics.workspace_bytes,
             IncrementalSetupHost(setup),
             IncrementalTransactionHost(transaction),
             fixed_bytes})
        output::Require(
            host.Append<std::byte>(bytes, unused),
            "Complete retained self-contact runtime exceeds host cap");

    RuntimeForecast result;
    result.transaction = transaction;
    result.retained_host_upper_bound = host.bytes();
    const auto scratch = std::max({
        dynamics.startup.peak_temporary_bytes,
        setup.peak_temporary_reservation_bytes,
        TransactionStartupScratch(transaction)});
    output::Require(
        host.Append<std::byte>(scratch, unused),
        "Complete self-contact runtime peak exceeds host cap");
    result.peak_host_upper_bound = host.bytes();
    for (const auto bytes : {
             dynamics.startup.device_bytes, transaction.device_bytes})
        output::Require(
            device.Append<std::byte>(bytes, unused),
            "Complete self-contact runtime exceeds device cap");
    result.device_bytes = device.bytes();
    return result;
}

CombinedRuntimeBudget ComposeCombinedBudget(
    std::size_t wall_retained_host,
    std::size_t wall_peak_host,
    std::size_t wall_device_bytes,
    std::size_t wall_publication_host,
    const SetupForecast& self_setup,
    const RuntimeForecast& self_contact,
    const tl::fea::ShellPhysicalScratchParticipationForecast&
        participation,
    std::size_t self_fixed,
    std::size_t host_limit,
    std::size_t device_limit) {
    output::Require(
        wall_retained_host >= wall_publication_host &&
            IncrementalTransactionHost(
                self_contact.transaction) >=
                self_contact.transaction.participation
                    .publication_host_bytes &&
            wall_peak_host >= wall_retained_host &&
            self_contact.peak_host_upper_bound >=
                self_contact.retained_host_upper_bound &&
            participation.publication_host_bytes &&
            participation.configured_issuer_host_bytes >=
                2 * sizeof(
                    tl::fea::ShellPhysicalScratchParticipation),
        "Wall+self component forecast partition is invalid");

    const auto wall_without_publication =
        wall_retained_host - wall_publication_host;
    const auto transaction_without_publication =
        IncrementalTransactionHost(self_contact.transaction) -
        self_contact.transaction.participation
            .publication_host_bytes;
    tl::util::BoundedArenaLayout host(host_limit);
    tl::util::BoundedArenaLayout device(device_limit);
    tl::util::ArenaRegion unused;
    for (const auto bytes : {
             wall_without_publication,
             IncrementalSetupHost(self_setup),
             transaction_without_publication,
             participation.publication_host_bytes,
             self_fixed})
        output::Require(
            host.Append<std::byte>(bytes, unused),
            "Complete retained wall+self runtime exceeds host cap");

    CombinedRuntimeBudget result;
    result.retained_host_upper_bound = host.bytes();
    const auto scratch = std::max(
        wall_peak_host - wall_retained_host,
        self_contact.peak_host_upper_bound -
            self_contact.retained_host_upper_bound);
    output::Require(
        host.Append<std::byte>(scratch, unused),
        "Complete wall+self runtime peak exceeds host cap");
    result.peak_host_upper_bound = host.bytes();
    output::Require(
        device.Append<std::byte>(wall_device_bytes, unused) &&
            device.Append<std::byte>(
                self_contact.transaction.device_bytes, unused),
        "Complete wall+self runtime exceeds device cap");
    result.device_bytes = device.bytes();
    return result;
}

}  // namespace crash::cases::vehicle_self_contact::detail
