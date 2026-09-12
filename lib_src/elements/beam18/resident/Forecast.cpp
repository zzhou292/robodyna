// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tl::fea::beam18 {
BatchReport Batch::Forecast(const BatchConfig& config, const Model& model,
    BatchForecast& output) noexcept {
  batch_detail::ArenaLayout layout;
  auto report = batch_detail::Plan(config, model, layout);
  if (!report) return report;
  const auto retained = model.owned_payload_bytes();
  if (retained < sizeof(Model)) {
    return {BatchStatus::ResourceLimit, "Beam18 retained model payload is inconsistent"};
  }
  util::BoundedArenaLayout host(config.limits.max_host_bytes);
  util::ArenaRegion ignored;
  if (!host.Append<unsigned char>(sizeof(Batch) + sizeof(Impl), ignored) ||
      !host.Append<unsigned char>(retained - sizeof(Model), ignored) ||
      !host.Append<unsigned char>(layout.bytes, ignored) ||
      !host.Append<unsigned char>(layout.staging_bytes, ignored) ||
      !host.Append<unsigned char>(layout.proof.bytes, ignored)) {
    return {BatchStatus::ResourceLimit, "Complete beam18 startup and live proof exceed host budget"};
  }
  output = {layout.bytes, host.bytes()};
  return {};
}
} // namespace tl::fea::beam18
