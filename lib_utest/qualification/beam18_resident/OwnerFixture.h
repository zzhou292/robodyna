// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/beam18/resident/Batch.h"
#include "lib_src/assembly/Beam18NodeContributions.h"
#include "lib_utest/qualification/beam18_force/TestSupport.h"
#include "lib_utest/qualification/rigid_assembly_owner/Fixture.h"
namespace beam18_resident_test {
namespace fe = tl::fea;
namespace b = fe::beam18;
struct OwnerFixture {
  rigid_assembly_owner_test::Fixture mechanics{false, true, .002, true};
  b::Model model;
  fe::Beam18NodeContributions beams;
  fe::NodalCoefficientLedger ledger;
  fe::rigid::NodalRigidPartAssemblyModel parts;
  fe::NodalRigidGroupModel plain;
  fe::NodalRigidAssemblyBinding binding;
  OwnerFixture();
  fe::NodalCinWitnessSource Witnesses() const {
    return {&mechanics.cin_model, mechanics.ranges.data(), mechanics.witnesses.data(),
        mechanics.ranges.size(), mechanics.witnesses.size()};
  }
  b::BatchConfig Configuration() const;
};
} // namespace beam18_resident_test
