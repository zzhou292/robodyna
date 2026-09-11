// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../type13_model/Fixture.h"
#include "../type13_recurrence/Fixture.h"
#include "lib_src/elements/type13/resident/Storage.h"
#include "lib_src/elements/type13/resident/Kinematics.h"
#include "lib_src/elements/type13/resident/ResultChecks.h"

namespace type13_resident_test {
namespace fe = tl::fea;
namespace t = fe::type13;
namespace detail = t::batch_detail;
struct Source {
  t::Model model;
  fe::NodalNodeDomain domain;
  fe::Type13NodeContributions contributions;
  bool Initialize(const t::ModelInput& input) {
    if (!model.Initialize(input)) {
      return false;
    }
    std::vector<fe::NodalDomainNode> nodes(model.global_node_count());
    for (std::size_t n = 0; n < model.node_count(); ++n) {
      const auto& node = model.nodes()[n];
      if (node.global_node != SIZE_MAX) {
        nodes[node.global_node] = {node.source_id,
          tl::math::fixed3::Scale(node.position_native, model.units().length_to_m)};
      }
    }
    return domain.Initialize({model.source_instance_id(), nodes.data(), nodes.size()},
                              fe::NodalDomainLimits::Vehicle()) &&
           contributions.Initialize(model, domain);
  }
};
inline t::BatchConfig Config(std::size_t nodes) {
  t::BatchConfig config;
  config.owner.owner_id = 11;
  config.owner.node_count = nodes;
  config.owner.fixed_dt = 0x1p-28;
  config.owner.has_rotations = true;
  config.owner.temporal_scheme = fe::NodalTemporalScheme::StaggeredHalfKickStart;
  config.owner.velocity_phase = fe::NodalVelocityPhase::Collocated;
  config.configuration_id = 71;
  config.qualification_id = 83;
  return config;
}
struct Startup {
  tl::util::HostArena arena;
  detail::ArenaLayout layout;
  detail::Storage header;
  t::BatchDiagnostics diagnostics;
  t::BatchReport Initialize(const t::BatchConfig& config, const Source& source) {
    t::BatchForecast forecast;
    const auto checked = t::Batch::Forecast(config, source.contributions, forecast);
    if (!checked) {
      return checked;
    }
    if (!detail::MakeLayout(source.model.property_count(), source.model.connection_count(),
                            config.limits, layout) || !arena.Initialize(layout.bytes)) {
      return {t::BatchStatus::ResourceLimit, "Qualification arena allocation failed"};
    }
    return detail::BuildStartup(config, source.contributions, arena, layout, header, diagnostics);
  }
};
inline void Exact(const t::Evaluation& a, const t::Evaluation& b) {
  const auto av = type13_recurrence_test::Values(a);
  const auto bv = type13_recurrence_test::Values(b);
  for (std::size_t i = 0; i < av.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_TRUE(fe::shell_startup_detail::SameBits(av[i], bv[i]));
  }
}
} // namespace type13_resident_test
