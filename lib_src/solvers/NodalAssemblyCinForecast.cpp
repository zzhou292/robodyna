// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FENodalStateStorage.h"
#include "NodalCinStorage.h"
#include "NodalRigidGroupStorage.h"
#include "NodalForceStageCaptureLayout.h"
#include "NodalStateLayout.h"
#include "nodal_seal/RowLayout.h"
#include <cmath>

namespace tl::fea {
NodalAssemblyCinForecast FENodalState::ForecastAssemblyCin(const NodalStateConfig& config,
    const NodalRigidAssemblyBinding& binding,const NodalCinStartup& cin,
    bool rotation_presence) noexcept {
  using namespace nodal_detail;
  NodalAssemblyCinForecast result;
  if (!config.max_nodes || config.max_nodes > MaxActiveNodalStateNodes ||
      !config.node_count || config.node_count > config.max_nodes ||
      !config.max_device_bytes || config.max_device_bytes > MaxActiveNodalStateDeviceBytes) {
    result.report = {NodalStatus::ResourceLimit,"Nodal capacity exceeds admitted limits"};
    return result;
  }
  if (config.temporal_scheme != NodalTemporalScheme::StaggeredHalfKickStart) {
    result.report = {NodalStatus::UnsupportedTemporalScheme,"Combined CIN owner requires staggered initialization"};
    return result;
  }
  if (!std::isfinite(config.fixed_dt) || !std::isfinite(config.minimum_dt) ||
      config.minimum_dt <= 0 || config.fixed_dt < config.minimum_dt ||
      !std::isfinite(config.timestep_safety) || config.timestep_safety <= 0 ||
      config.timestep_safety >= 1 || !(.5*config.fixed_dt > 0) ||
      !(.5*config.fixed_dt < config.fixed_dt)) {
    result.report = {NodalStatus::InvalidInput,"Invalid fixed-step capacity descriptor"};
    return result;
  }
  RigidStorageLayout rigid;
  result.report = ForecastRigidStorage(binding,config,rigid);
  if (result.report.status != NodalStatus::Ok) return result;
  CinLayout attachment;
  result.report = ForecastCinStorage(cin,config,attachment,binding.groups().size());
  if (result.report.status != NodalStatus::Ok) return result;
  if (!binding.domain()->SharesStorage(*cin.model->domain())) {
    result.report = {NodalStatus::InvalidInput,"Combined owner sources do not share the physical domain"};
    return result;
  }
  const auto groups = binding.groups().size();
  const ForceStageCaptureLayout capture{config.node_count,groups};
  const auto capture_values = config.capture_force_stage_accelerations ? capture.values() : 0;
  StateLayout layout;
  if (!layout.Initialize(config.node_count,true,rigid::GroupStateValues*groups,
      rigid.device_bytes,capture_values,nodal_seal::ControlBytes(sizeof(Control),config.node_count),config.max_device_bytes,
      attachment.state_values,attachment.device_bytes,rotation_presence)) {
    result.report = {NodalStatus::ResourceLimit,"Combined owner device layout exceeds its cap"};
    return result;
  }
  std::size_t complete = 0;
  if (!CinOwnerHostFits(attachment.host_bytes,rigid.host_bytes,layout.accepted.count,
      layout.fixed.count,sizeof(Impl),cin.limits.max_host_bytes,&complete)) {
    result.report = {NodalStatus::ResourceLimit,"Complete CIN owner host payload exceeds limits"};
    return result;
  }
  // These three exact retained charges were already checked by ForecastCinStorage.
  const auto source = cin.model->forecast();
  const auto source_bytes = source.model_payload_bytes + source.domain_payload_bytes +
                            source.post_kinchk_payload_bytes;
  const auto scratch = util::SourceIdentityIndex<16>::Bytes(cin.witness_count);
  if (source_bytes > complete || scratch > complete - source_bytes) {
    result.report = {NodalStatus::ResourceLimit,"Combined owner footprint partition overflows"};
    return result;
  }
  result.device_bytes = layout.bytes;
  result.source_host_bytes = source_bytes;
  result.owner_host_bytes = complete - source_bytes - scratch;
  result.startup_scratch_bytes = scratch;
  result.startup_host_bytes = complete;
  result.report = {NodalStatus::Ok,"Combined owner capacity prepared; physical input admission remains required"};
  return result;
}
} // namespace tl::fea
