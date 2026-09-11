// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TiedSearchDriver.h"
#include "../TiedSearchBounds.h"
#include "lib_src/collision/HydroelasticBroadphase.cuh"

namespace tl::constraints::tied_shell::driver_detail {
struct Scratch {
  std::size_t sort = 0;
  std::size_t scan = 0;
};
SearchDriverReport CheckCounts(const SearchDriverInput&, const SearchDriverLimits&) noexcept;
SearchDriverReport CheckValues(const SearchDriverInput&);
SearchDriverReport Budget(const SearchDriverInput&, const SearchDriverLimits&, Scratch,
                          SearchDriverBudget&) noexcept;
std::size_t FixedDeviceBytes(const SearchDriverInput&, Scratch) noexcept;
SearchDriverReport PrepareBounds(const SearchDriverInput&, std::vector<NativeSearchBounds>&,
                                 std::vector<double>&);
SearchDriverReport Reduce(const SearchDriverInput&, std::vector<CollisionPair>&,
                         const std::vector<NativeSearchBounds>&, SearchDriverResult&);
inline SearchDriverReport Error(SearchDriverStatus status, const char* message,
    std::size_t secondary = SIZE_MAX, std::size_t master = SIZE_MAX,
    Status numerical = Status::Success) noexcept {
  return {status, numerical, secondary, master, message};
}
} // namespace tl::constraints::tied_shell::driver_detail
