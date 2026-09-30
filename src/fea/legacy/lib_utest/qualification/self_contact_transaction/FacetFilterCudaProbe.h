// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>
namespace facet_filter_cuda_probe {
std::size_t Allocations() noexcept;
std::size_t Copies() noexcept;
void FailNextHostToDeviceCopy() noexcept;
}  // namespace facet_filter_cuda_probe
