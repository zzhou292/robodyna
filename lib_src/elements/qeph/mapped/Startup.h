// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../QephBatchStorage.h"
#include "../../ShellMappedStartup.h"

namespace tl::fea::qeph::mapped {
struct Forecast {
  batch_detail::Layout device;
  shell_physical_owner::ProofLayout proof;
  std::size_t host_bytes=0,section_device_bytes=0;
};
BatchReport MakeForecast(const QephBatchConfig&,const ShellPhysicalBinding&,
    const NodalCinWitnessSource&,const ShellBatchFailureLimits&,std::size_t,Forecast&) noexcept;
BatchReport BuildModel(const QephBatchConfig&,const ShellPhysicalBinding&,
    batch_detail::Storage&) noexcept;
} // namespace tl::fea::qeph::mapped
