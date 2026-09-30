// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ResultValues.h"
#include "lib_src/elements/solids/Model.h"
#include <memory>

namespace solid_resident_test {
// Independent complete native states. Only actual owner positions/velocities
// enter each native packet; no production history or coefficient seeds do.
class NativeTrajectory {
 public:
  NativeTrajectory();
  ~NativeTrajectory();
  bool Initialize(const s::Model&,tl::math::Vec3 velocity);
  bool Evaluate(const std::vector<double>& position,const std::vector<double>& velocity,
      double base_time,double dt,std::uint64_t epoch);
  void Compare(const Results&,double time,std::uint64_t epoch) const;
  void Accept();
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace solid_resident_test
