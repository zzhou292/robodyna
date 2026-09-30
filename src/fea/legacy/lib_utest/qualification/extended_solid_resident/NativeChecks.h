// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "OwnerFixture.h"
#include <memory>
namespace extended_resident_test {
class NativeChecks {
  struct State;
  std::unique_ptr<State> state_;
 public:
  NativeChecks();
  ~NativeChecks();
  bool Initialize(const s::Model&,const tl::material::law90::PreparationInput&,fe::solid18::Vec3);
  bool Evaluate(const std::vector<double>&,const std::vector<double>&,double,double,std::uint64_t);
  bool Compare(const Results&,bool candidate);
  void Commit();
};
} // namespace extended_resident_test
