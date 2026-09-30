#pragma once

#include "VehicleSelfContactStartup.h"

namespace crash::cases::vehicle_self_contact::detail {

tlfea::contact::SelfContactTransactionConfig TransactionConfig(RuntimeConfig,
    const tl::fea::NodalStamp&, const tl::fea::ShellPhysicalPublicationIdentity&) noexcept;
// Select one exact component footprint using the owner's typed initialization route.
bool TransactionAllocationsMatch(RuntimeConfig,
    tlfea::contact::SelfContactFacetFilterInitialization,
    const tlfea::contact::SelfContactTransactionForecast& requested,
    const tlfea::contact::SelfContactTransactionAllocationInfo& actual,
    const tlfea::contact::SelfContactTransactionForecast* cpu_fallback = nullptr) noexcept;
// The initialized self component has already passed TransactionAllocationsMatch.
std::size_t EffectiveCombinedDeviceBytes(std::size_t combined,
    std::size_t requested_self, std::size_t initialized_self);

std::size_t IncrementalSetupHost(
    const SetupForecast&);
bool TransactionChargesBroadphaseBacking() noexcept;
std::size_t IncrementalTransactionHost(
    const tlfea::contact::SelfContactTransactionForecast&);
std::size_t TransactionStartupScratch(
    const tlfea::contact::SelfContactTransactionForecast&);

RuntimeForecast ComposeForecast(
    const vehicle_dynamics::Forecast&, const SetupForecast&,
    const tlfea::contact::SelfContactTransactionForecast&,
    std::size_t fixed_bytes, RuntimeLimits);

struct CombinedRuntimeBudget {
    std::size_t retained_host_upper_bound = 0;
    std::size_t peak_host_upper_bound = 0;
    std::size_t device_bytes = 0;
};

CombinedRuntimeBudget ComposeCombinedBudget(
    std::size_t wall_retained_host,
    std::size_t wall_peak_host,
    std::size_t wall_device_bytes,
    std::size_t wall_publication_host,
    const SetupForecast&, const RuntimeForecast&,
    const tl::fea::ShellPhysicalScratchParticipationForecast&,
    std::size_t self_fixed,
    std::size_t host_limit,
    std::size_t device_limit);

}  // namespace crash::cases::vehicle_self_contact::detail
