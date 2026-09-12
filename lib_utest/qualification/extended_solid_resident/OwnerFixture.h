// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include "lib_utest/qualification/solid_resident/Fixture.h"
#include "lib_utest/qualification/solid_resident/PublicationPeer.h"
namespace extended_resident_test {
struct OwnerFixture {
  solid_resident_test::Fixture legacy;
  s::Model model;
  fe::NodalCoefficientLedger ledger;
  fe::rigid::NodalRigidPartAssemblyModel parts;
  fe::NodalRigidGroupModel plain;
  fe::NodalRigidAssemblyBinding binding;
  tl::material::law90::PreparationInput foam_input;
  double rear_x[3]{0,.2,.4},rear_y[3]{270e6,350e6,450e6};
  double foam_x[3]{0,.2,.4},foam_y[3]{0,10e6,50e6};
  explicit OwnerFixture(bool analytic44 = false);
  auto& Mechanics() { return legacy.mechanics; }
  auto Witnesses() const { return legacy.Witnesses(); }
  s::BatchConfig Configuration() const {
    auto config=legacy.Configuration();
    config.profile=s::BatchProfile::PhysicalCinExtendedLaw44Law90V2;
    config.owner.fixed_dt=1e-8;
    return config;
  }
};
struct Results {
  std::vector<s::Result18> old18;
  std::vector<s::Result24> old24;
  std::vector<s::Result6z> old6z;
  std::vector<s::Result18Law44> rear;
  std::vector<s::Result18Law90> foam;
  explicit Results(const s::Model& m):old18(m.solid18().size()),old24(m.solid24().size()),
    old6z(m.solid6z().size()),rear(m.solid18_law44().size()),foam(m.solid18_law90().size()) {}
  s::ResultBuffers Buffers() {
    return {old18.empty()?nullptr:old18.data(),old18.size(),old24.empty()?nullptr:old24.data(),old24.size(),
      old6z.empty()?nullptr:old6z.data(),old6z.size(),rear.empty()?nullptr:rear.data(),rear.size(),
      foam.empty()?nullptr:foam.data(),foam.size()};
  }
};
} // namespace extended_resident_test
