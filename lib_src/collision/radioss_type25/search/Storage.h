// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../RadiossType25Search.h"
#include "Layout.h"
namespace tlfea::contact::radioss_type25::search {
struct Maintenance::Impl {
  ~Impl();
  Source source;
  Limits limits;
  detail::Layout layout;
  units_detail::Factors factors;
  detail::Device device;
  detail::DeviceControl control;
  FailureInfo failure;
  void* arena = nullptr;
  cudaStream_t stream = nullptr;
  std::uint64_t identity = 0, sequence = 0, generation = 0;
  QueryStamp accepted_stamp, staged_stamp;
  unsigned accepted = 0;
  bool usable = true, reference = false, pending = false;
  Status Check(const Current&,bool capture) const noexcept;
  Status Execute(const Current&,unsigned slab,bool capture,double previous_dt,bool force_sort) noexcept;
  bool OutputDisjoint(const void*, std::size_t, const Current&) const noexcept;
  Status Result(Status, const Current* = nullptr, std::size_t row = SIZE_MAX) noexcept;
};
} // namespace tlfea::contact::radioss_type25::search
