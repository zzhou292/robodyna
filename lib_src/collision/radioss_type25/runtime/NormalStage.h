// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Launch.h"
namespace tlfea::contact::radioss_type25::runtime_detail {
// Each launch group is stream ordered; Transaction fences and checks Control
// before consuming the next group's count or binding any staged normal view.
cudaError_t CountBeforeNormals(Device,lifecycle::Input,const units_detail::Factors&,cudaStream_t) noexcept;
cudaError_t EmitOptimized(Device,lifecycle::Input,const units_detail::Factors&,std::size_t,cudaStream_t) noexcept;
cudaError_t UpdateNormals(Device,lifecycle::Input,const units_detail::Factors&,unsigned,unsigned,std::size_t,cudaStream_t) noexcept;
cudaError_t CountAfterNormals(Device,lifecycle::Input,const units_detail::Factors&,cudaStream_t) noexcept;
} // namespace tlfea::contact::radioss_type25::runtime_detail
