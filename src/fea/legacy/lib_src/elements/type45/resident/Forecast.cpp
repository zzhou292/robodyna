// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tl::fea::type45 {
BatchReport Batch::Forecast(const BatchConfig& config,const Model& model,BatchForecast& output) noexcept {
  resident_detail::ArenaLayout layout;
  auto report=resident_detail::Plan(config,model,layout);
  if(!report) return report;
  if(!resident_detail::ModelOutputDisjoint(model,&output,sizeof(output)) ||
      !trial_identity::Disjoint(&output,sizeof(output),&config,sizeof(config)))
    return {BatchStatus::InvalidInput,"Joint forecast output overlaps its source or configuration"};
  const auto retained=model.owned_payload_bytes();
  if(retained<sizeof(Model)) return {BatchStatus::ResourceLimit,"Joint retained payload is inconsistent"};
  util::BoundedArenaLayout host(config.limits.max_host_bytes);
  util::ArenaRegion ignored;
  if(!host.Append<unsigned char>(sizeof(Batch)+sizeof(Impl),ignored) ||
      !host.Append<unsigned char>(retained-sizeof(Model),ignored) ||
      !host.Append<unsigned char>(layout.staging_bytes,ignored) ||
      !host.Append<unsigned char>(layout.bytes>layout.proof.bytes?layout.bytes:layout.proof.bytes,ignored))
    return {BatchStatus::ResourceLimit,"Complete joint startup and owner proof exceed host budget"};
  const auto backing=retained-sizeof(Model);
  output={layout.bytes,host.bytes(),backing,host.bytes()-backing};return {};
}
} // namespace tl::fea::type45
