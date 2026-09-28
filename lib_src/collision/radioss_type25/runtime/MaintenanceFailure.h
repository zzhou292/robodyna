// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "lib_src/collision/RadiossType25Search.h"
namespace tlfea::contact::radioss_type25::runtime_detail {
// Preserve the owning search result before common rollback revokes its trial.
// The old top-level status mapping is retained; the exact leaf is additional
// diagnostic evidence, never a substitute for candidate/physical authority.
inline TransactionReport MaintenanceFailure(TransactionDiagnostics& diagnostics,
    const search::Maintenance& maintenance,search::Status status,
    MaintenanceOperation operation,bool force_sort,const char* message) noexcept {
  diagnostics.maintenance_failure_available=true;
  diagnostics.maintenance_operation=operation;
  diagnostics.maintenance_force_sort=force_sort;
  diagnostics.maintenance_failure=maintenance.last_failure();
  return {status==search::Status::DeviceFailure?TransactionStatus::DeviceFailure:
      TransactionStatus::NumericalFailure,message,
      diagnostics.maintenance_failure.row_available?diagnostics.maintenance_failure.input_row:SIZE_MAX};
}
}
