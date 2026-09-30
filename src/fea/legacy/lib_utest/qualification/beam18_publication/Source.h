// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_utest/qualification/physical_publication/Fixture.h"
#include "lib_src/assembly/Beam18NodeContributions.h"
namespace beam18_publication_test {
namespace fe = tl::fea;
namespace b = fe::beam18;
namespace existing = physical_publication_test;
// Append a typed beam producer to the unchanged complete source fixture. No
// mesh, material, CIN roster or other element source is reconstructed here.
struct Source {
  b::Model model;
  fe::Beam18NodeContributions contribution;
  fe::NodalCoefficientLedger ledger;
  fe::rigid::NodalRigidPartAssemblyModel parts;
  fe::NodalRigidGroupModel plain;
  fe::NodalRigidAssemblyBinding rigid;
  fe::ShellPhysicalBinding physical;
  bool Initialize(existing::Fixture&);
};
} // namespace beam18_publication_test
