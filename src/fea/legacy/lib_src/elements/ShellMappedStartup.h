// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellPhysicalOwner.h"
#include "ShellBatchPlasticityStorage.h"
#include "../assembly/ShellPhysicalBinding.h"

namespace tl::fea::shell_mapped_detail {
NodalReport Validate(const NodalStamp&,const ShellBatchStartup&,
    const ShellPhysicalBinding&,const NodalCinWitnessSource&) noexcept;
// The physical handle already owns the exact failure/catalog backing. Only
// sidecar storage and its copied handle are charged by this forecast.
bool ForecastSections(const ShellPhysicalBinding&,ShellBindingFamily,std::size_t,
    std::size_t device_cap,std::size_t host_cap,const ShellBatchFailureLimits&,
    std::size_t& host_bytes,std::size_t& device_bytes) noexcept;
// No new coefficient reduction: copy one complete authoritative ledger into
// existing family diagnostic arrays. Zero scalar J is a valid source value.
template<class Model> bool CopyLedger(const ShellPhysicalBinding& physical,Model& model) noexcept {
  const auto nodes=physical.domain()->nodes();
  const auto values=physical.coefficients()->nodes();
  double kinetic=0;
  for (std::size_t node=0;node<nodes.size();++node) {
    const auto& c=values[node].coefficients;
    model.initial_position[node]=nodes[node].position;
    model.mass[node]=c.mass;
    model.inertia[node]=c.isotropic_inertia;
    model.physical[node]=c.shell.physical_inertia;
    model.added[node]=c.shell.added_inertia;
    if (model.config.startup.kind!=ShellBatchStartupKind::ReferenceConstrainedUniformTranslation &&
        !shell_startup_detail::AddInitialTranslationKinetic(c.mass,
        model.config.startup.uniform_velocity,kinetic)) return false;
  }
  return true;
}
} // namespace tl::fea::shell_mapped_detail
