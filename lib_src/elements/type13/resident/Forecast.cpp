// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tl::fea::type13::batch_detail {
BatchReport SourceForecast(const BatchConfig& config,
                            const Type13NodeContributions& source,
                            std::size_t private_bytes, BatchForecast& output) noexcept {
  if (!source.prepared() || !source.model() || !source.domain()) {
    return {BatchStatus::InvalidInput, "Prepared TYPE13 endpoint/domain source required"};
  }
  const auto& model = *source.model();
  batch_detail::ArenaLayout layout;
  if (config.owner.node_count > config.limits.max_nodes ||
      !batch_detail::MakeLayout(model.property_count(), model.connection_count(),
                               config.limits, layout)) {
    return {BatchStatus::ResourceLimit, "TYPE13 counts or device bounds exceed scope"};
  }
  const auto source_bytes = source.owned_payload_bytes();
  if (source_bytes < sizeof(Type13NodeContributions)) {
    return {BatchStatus::ResourceLimit, "TYPE13 retained payload is inconsistent"};
  }
  util::BoundedArenaLayout host(config.limits.max_host_bytes);
  util::ArenaRegion ignored;
  // The embedded source handle is already counted in Impl. Its backing owns
  // the model and domain; neither is retained or charged a second time here.
  if (!host.Append<unsigned char>(private_bytes, ignored) ||
      !host.Append<unsigned char>(source_bytes - sizeof(Type13NodeContributions), ignored) ||
      !host.Append<unsigned char>(layout.bytes, ignored) ||
      !host.Append<Evaluation>(model.connection_count(), ignored)) {
    return {BatchStatus::ResourceLimit, "Complete TYPE13 startup exceeds host byte cap"};
  }
  output = {layout.bytes, host.bytes()};
  return {};
}
} // namespace tl::fea::type13::batch_detail
namespace tl::fea::type13 {
BatchReport Batch::Forecast(const BatchConfig& config,
    const Type13NodeContributions& source, BatchForecast& output) noexcept {
  BatchForecast next;
  const auto forecast = batch_detail::SourceForecast(config, source,
      sizeof(Batch) + sizeof(Impl), next);
  if (!forecast) return forecast;
  const auto checked = batch_detail::SourcePreflight(config, source);
  if (!checked) return checked;
  output = next;
  return {};
}
} // namespace tl::fea::type13
