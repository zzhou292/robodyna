// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../activity_operands/State.h"
#include "lib_src/elements/publication/PhysicalActivitySnapshot.h"
namespace tlfea::contact::radioss_type25::runtime_detail {
// Per-interface observer avoids sharing a cached physical proof across public
// calls. Both interfaces observe the same common owner; neither publishes it.
struct ActivityRuntime {
  tl::fea::PhysicalActivitySnapshot snapshot;
  tl::fea::PhysicalAcceptedActivityReceipt accepted;
  tl::fea::PhysicalPreparedActivityReceipt prepared;
  activity_operands::State operands;
};
cudaError_t ApplyActivitySecondary(const double*,lifecycle::Secondary*,std::size_t,cudaStream_t) noexcept;
TransactionReport ActivityReport(tl::fea::PhysicalActivityReport) noexcept;
}
