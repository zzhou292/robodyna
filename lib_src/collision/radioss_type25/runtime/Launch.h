// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Layout.h"
#include "lib_src/solvers/NodalCinRuntime.h"
#include <cuda_runtime_api.h>
namespace tlfea::contact::radioss_type25::runtime_detail {
cudaError_t QueryScratch(std::size_t rows,std::size_t candidates,std::size_t&) noexcept;
cudaError_t ValidateCurrent(Device,const tl::fea::NodalAssemblyView&,cudaStream_t) noexcept;
cudaError_t ConvertInventory(Device,const candidates::Pair*,const std::uint64_t*,std::size_t,cudaStream_t) noexcept;
cudaError_t Prepare(Device,lifecycle::Input,const units_detail::Factors&,cudaStream_t) noexcept;
cudaError_t CountCandidates(Device,lifecycle::Input,const units_detail::Factors&,cudaStream_t) noexcept;
cudaError_t Complete(Device,lifecycle::Input,const units_detail::Factors&,std::size_t,cudaStream_t) noexcept;
cudaError_t Order(Device,std::size_t,cudaStream_t) noexcept;
cudaError_t Respond(Device,lifecycle::Input,const TransactionConfig&,const units_detail::Factors&,
    double,unsigned,std::size_t,std::size_t,cudaStream_t) noexcept;
cudaError_t Gather(Device,assembly::Schedule,assembly::Incidence,const tl::fea::NodalAssemblyView&,
    const tl::fea::NodalCinAssemblyView&,cudaStream_t) noexcept;
cudaError_t Apply(Device,const tl::fea::NodalAssemblyView&,const tl::fea::NodalCinAssemblyView&,cudaStream_t) noexcept;
} // namespace tlfea::contact::radioss_type25::runtime_detail
