// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../MeasurementValues.h"
#include <cfloat>
namespace tl::fea::solids::batch_detail::measurement {
TL_BRICK_HD inline Control Begin(const Storage& state,BatchDiagnostics identity,bool initial) noexcept {
  Control next{};
  if (initial) {
    identity.source_instance_id = state.source_instance_id;
    identity.owner_id = state.config.owner.owner_id;
    identity.configuration_id = state.config.configuration_id;
    identity.qualification_id = state.config.qualification_id;
    identity.phase = BatchPhase::Accepted;
  }
  identity.controlled_packet_blocks=state.controlled.blocks;
  identity.controlled_worker_slots=state.controlled.workspace_count;
  identity.minimum_native_dt_s = DBL_MAX;
  next.diagnostics = identity;
  return next;
}
} // namespace tl::fea::solids::batch_detail::measurement
