// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellMappedStartup.h"
#include <cmath>

namespace tl::fea::shell_mapped_detail {
NodalReport Validate(const NodalStamp& owner,const ShellBatchStartup& startup,
    const ShellPhysicalBinding& physical,const NodalCinWitnessSource& source) noexcept {
  if (!physical.prepared() || !source.model || !source.model->prepared() ||
      !owner.owner_id || !owner.has_rotations || owner.epoch || owner.time!=0 ||
      owner.velocity_time!=0 || owner.reactions_valid || !std::isfinite(owner.fixed_dt) ||
      owner.fixed_dt<=0 || owner.temporal_scheme!=NodalTemporalScheme::StaggeredHalfKickStart ||
      owner.velocity_phase!=NodalVelocityPhase::Collocated ||
      owner.node_count!=physical.domain()->node_count() ||
      !shell_startup_detail::ValidStartup(startup,true)) {
    return {NodalStatus::InvalidInput,"Mapped shells require a complete physical scope and fresh staggered owner"};
  }
  if (source.range_count!=source.model->rows().count || !source.range_count ||
      source.range_count>NodalCinLimits{}.max_attachments || !source.witness_count ||
      source.witness_count>NodalCinLimits{}.max_witnesses || !source.ranges || !source.witnesses ||
      !source.model->domain()->SharesStorage(*physical.domain())) {
    return {NodalStatus::InvalidInput,"Mapped shell CIN source/domain differs or exceeds scope"};
  }
  if (physical.catalog()->execution_sections()!=bool(physical.execution())) {
    return {NodalStatus::InvalidInput,"Mapped shell execution role authority is incomplete"};
  }
  return {NodalStatus::Ok,"OK"};
}
bool ForecastSections(const ShellPhysicalBinding& physical,ShellBindingFamily family,std::size_t count,
    std::size_t device_cap,std::size_t host_cap,const ShellBatchFailureLimits& limits,
    std::size_t& host_bytes) noexcept {
  ShellSectionCounts counts;
  if (!physical.prepared() || !physical.catalog()->Counts(family,&counts) ||
      (counts.law44_nip1 && family!=ShellBindingFamily::T3)) return false;
  return shell_batch_plasticity_detail::HostStorage::ForecastFailureSections(count,
      physical.catalog()->curve_point_count(),sizeof(ShellBatchFailureBinding),
      device_cap,host_cap,limits,host_bytes,counts.law44_nip1!=0);
}
} // namespace tl::fea::shell_mapped_detail
