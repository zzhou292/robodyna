#pragma once

#include "VehicleSelfContactStartup.h"

namespace crash::cases::vehicle_self_contact::detail {

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

}  // namespace crash::cases::vehicle_self_contact::detail
