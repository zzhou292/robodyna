// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../solid_model/TestSupport.h"
#include "lib_src/elements/solids/resident/Arena.h"

namespace solid_resident_test {
namespace fe = tl::fea;
namespace s = fe::solids;
namespace detail = s::batch_detail;
inline s::BatchConfig Config(std::size_t nodes) {
  s::BatchConfig config;
  config.owner.owner_id = 501;
  config.owner.node_count = nodes;
  config.owner.fixed_dt = 1e-8;
  config.owner.has_rotations = true;
  config.owner.temporal_scheme = fe::NodalTemporalScheme::StaggeredHalfKickStart;
  config.owner.velocity_phase = fe::NodalVelocityPhase::Collocated;
  config.configuration_id = 502;
  config.qualification_id = 503;
  config.profile = s::BatchProfile::PhysicalCinV1;
  config.cin_attachment_count = 1;
  config.cin_witness_count = 1;
  return config;
}
template<class Traits> void Fill(const typename Traits::Parent& parent,
    typename Traits::Interval& interval, double scale = 1) {
  for (unsigned n = 0; n < Traits::nodes; ++n) {
    auto x = parent.reference.input().position_m[n];
    x.x *= scale;
    Traits::Node(interval, n, x, {.01 * double(n), -.02 * double(n), .003 * double(n)});
  }
}
} // namespace solid_resident_test
