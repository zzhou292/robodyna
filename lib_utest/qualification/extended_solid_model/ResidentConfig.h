// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solids/resident/Batch.h"
namespace extended_model_test {
inline tl::fea::solids::BatchConfig ResidentConfig(std::size_t nodes) {
  tl::fea::solids::BatchConfig config;
  config.owner.owner_id = 501; config.owner.node_count = nodes;
  config.owner.fixed_dt = 1e-8; config.owner.has_rotations = true;
  config.owner.temporal_scheme = tl::fea::NodalTemporalScheme::StaggeredHalfKickStart;
  config.owner.velocity_phase = tl::fea::NodalVelocityPhase::Collocated;
  config.configuration_id = 502; config.qualification_id = 503;
  config.profile = tl::fea::solids::BatchProfile::PhysicalCinV1;
  config.cin_attachment_count = 1; config.cin_witness_count = 1;
  return config;
}
}
