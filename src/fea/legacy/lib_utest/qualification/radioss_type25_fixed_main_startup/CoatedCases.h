// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "GeneralCases.h"
#include <utility>
namespace type25_startup_test {
// Explicit declared source roles, not a inferred shell/solid membership result.
inline Case Coated(Case source, unsigned shift = 0) {
  source.profile = s::Profile::ResolvedShellSides;
  source.topology = s::TopologyPolicy::NativeResolvedShellSides;
  constexpr s::ShellSideRole roles[]{s::ShellSideRole::Ordinary,
      s::ShellSideRole::CoatingForward, s::ShellSideRole::CoatingReversed};
  for (std::size_t i = 0; i < source.primary.size(); ++i)
    source.primary[i].side_role = roles[(i + shift) % 3];
  return source;
}
struct CoatedBuilt {
  tl::util::HostArena output, scratch;
  s::Snapshot startup;
  s::Forecast forecast;
  s::Report report;
  explicit CoatedBuilt(const Case& source) {
    const auto input = source.Input();
    forecast = s::Preflight(input);
    if (forecast.status != s::Status::Ok || !output.Initialize(forecast.output_bytes) ||
        !scratch.Initialize(forecast.scratch_bytes))
      throw std::runtime_error("Resolved-shell startup fixture allocation failed");
    report = s::BuildStarter(input, {}, output, scratch, &startup);
    if (report.status != s::Status::Ok)
      throw std::runtime_error("Resolved-shell startup fixture rejected");
  }
};
} // namespace type25_startup_test
