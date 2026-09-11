// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../nodal_coefficients/SolidFixture.h"
#include "../qbat_catalog/Fixture.h"
#include "lib_src/elements/ShellBatchPublication.h"
#include "lib_src/assembly/ShellPhysicalBinding.h"
#include "lib_src/elements/solids/Model.h"
#include "lib_src/constraints/NodalRigidAssemblyBinding.h"
#include "lib_src/constraints/tied_shell/TiedCinAttachmentModel.h"
#include "lib_src/solvers/NodalCinRuntime.h"

namespace physical_publication_test {
namespace fe = tl::fea;
namespace tied = tl::constraints::tied_shell;
inline constexpr std::uint64_t Configuration = 718,Qualification = 719;
inline constexpr double H = 0x1p-28;
using qbat_binding_test::Bits;
struct Fixture {
  coefficient_test::SolidFixture source;
  fe::NodalNodeDomain domain;
  fe::ShellNodeMap shells;
  fe::type25::Model welds;
  fe::type13::Model beams;
  fe::Type13NodeContributions beam_coefficients;
  fe::solids::Model solids;
  fe::ElementMassContributions point;
  fe::NodalCoefficientLedger ledger;
  fe::rigid::NodalRigidPartTopology topology;
  fe::rigid::NodalRigidPartAssemblyModel parts;
  fe::NodalRigidGroupModel plain;
  fe::NodalRigidAssemblyBinding rigid;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  fe::ShellPhysicalBinding physical;
  tied::PostKinChkResult post;
  tied::TiedCinAttachmentModel cin;
  std::array<tied::cin::WitnessRange,1> ranges{{{0,3}}};
  std::array<tied::cin::ActiveWitness,3> witnesses;
  std::vector<double> x,v,w,q,m,j,im,ij;
  std::vector<std::uint8_t> fixed,rotation_fixed,present;
  Fixture();
  void PrepareSources();
  void PrepareConstraints();
  void PrepareMaterials();
  fe::NodalCinWitnessSource WitnessSource() const {
    return {&cin,ranges.data(),witnesses.data(),ranges.size(),witnesses.size()};
  }
  fe::NodalCinStartup CinStartup() const {
    return {&cin,m.data(),j.data(),ranges.data(),witnesses.data(),witnesses.size(),Qualification};
  }
  fe::NodalStateConfig OwnerConfig() const {
    fe::NodalStateConfig c;
    c.node_count = domain.node_count();
    c.fixed_dt = H;
    c.max_device_bytes = 4u << 20;
    c.temporal_scheme = fe::NodalTemporalScheme::StaggeredHalfKickStart;
    c.rigid_limits = fe::NodalRigidOwnerLimits::VehicleAssembly();
    return c;
  }
  fe::ShellPhysicalPublicationIdentity Identity() const { return {Configuration,Qualification,{}}; }
};
} // namespace physical_publication_test
