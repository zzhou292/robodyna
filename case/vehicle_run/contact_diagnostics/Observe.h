#pragma once
#include "Values.h"
namespace tlfea::contact {struct SelfContactTransactionDiagnostics;}
namespace crash::cases::vehicle_run::contact_diagnostics {
Snapshot Copy(const tlfea::contact::SelfContactTransactionDiagnostics&) noexcept;
// Mismatched diagnostics become unavailable; they cannot reject mechanics.
Snapshot Committed(const tlfea::contact::SelfContactTransactionDiagnostics&,
    std::uint64_t owner,std::uint64_t base_epoch,std::uint64_t attempt) noexcept;
} // namespace crash::cases::vehicle_run::contact_diagnostics
