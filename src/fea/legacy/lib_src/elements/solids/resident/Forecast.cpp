// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
namespace tl::fea::solids {
BatchReport Batch::Impl::Resolve(const BatchConfig& config,const Model& model,
    batch_detail::ArenaLayout& output,BatchForecast& forecast) noexcept {
  auto attempt=config;const bool controlled=config.profile==BatchProfile::PhysicalCinSourceControlsV3;
  auto blocks=config.limits.max_controlled_packet_blocks;
  if(controlled&&(blocks!=4&&blocks!=8&&blocks!=16&&blocks!=32))
    return {BatchStatus::InvalidInput,"Controlled packet ceiling must be4,8,16 or32"};
  if(controlled&&blocks>8){
    // Do not add idle CTAs beyond the last enabled packet. Default8 behavior is
    // retained; requests above8 only expand where another packet can use them.
    if(const auto* selection=model.control_selection()){
      std::size_t span=0;for(std::size_t p=0;p<selection->packets().size();++p)
        if(selection->packets()[p].source.icontrol)span=p+1;
      while(blocks>8&&span<=blocks/2)blocks/=2;
    }
  }
  for(;;){
    attempt.limits.max_controlled_packet_blocks=blocks;
    batch_detail::ArenaLayout layout;auto report=batch_detail::Plan(attempt,model,layout);
    if(report){
      const auto retained=model.owned_payload_bytes();
      if(retained<sizeof(Model))return {BatchStatus::ResourceLimit,"Solid retained model payload is inconsistent"};
      util::BoundedArenaLayout host(config.limits.max_host_bytes);util::ArenaRegion ignored;
      if(host.Append<unsigned char>(sizeof(Batch)+sizeof(Impl),ignored)&&
         host.Append<unsigned char>(retained-sizeof(Model),ignored)&&
         host.Append<unsigned char>(layout.bytes,ignored)&&
         host.Append<unsigned char>(layout.staging_bytes,ignored)&&
         host.Append<unsigned char>(layout.proof.bytes,ignored)){
        BatchForecast next{layout.bytes,host.bytes(),layout.controlled.blocks,layout.controlled.workspace.count};
        output=layout;forecast=next;return {};
      }
      report={BatchStatus::ResourceLimit,"Complete solid startup and live proof exceed host budget"};
    }
    if(!controlled||blocks<=4||report.status!=BatchStatus::ResourceLimit)return report;
    blocks/=2;
  }
}
BatchReport Batch::Forecast(const BatchConfig& config,const Model& model,BatchForecast& output) noexcept {
  batch_detail::ArenaLayout layout;BatchForecast next;
  const auto report=Impl::Resolve(config,model,layout,next);
  if(report)output=next;return report;
}
} // namespace tl::fea::solids
