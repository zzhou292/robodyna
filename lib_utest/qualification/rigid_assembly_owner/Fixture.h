// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../nodal_coefficients/SolidFixture.h"
#include "../tied_cin_attachment/Fixture.h"
#include "lib_src/solvers/NodalRigidGroupStorage.h"
#include "lib_src/solvers/NodalCinStorage.h"

namespace rigid_assembly_owner_test {
namespace fe = tl::fea;
namespace r = fe::rigid;
namespace nd = fe::nodal_detail;
namespace tied = tl::constraints::tied_shell;
namespace cin = tied::cin;
using Code = fe::NodalStatus;
inline constexpr double H = 0x1p-20;
inline constexpr std::uint64_t Qualification = 718;
struct Fixture {
  coefficient_test::SolidFixture source;
  cin_test::Fixture cin_source;
  fe::NodalNodeDomain domain;
  fe::ShellNodeMap shells;
  fe::SolidNodeContributions solids;
  fe::ElementMassContributions point;
  fe::type25::Model springs;
  fe::NodalCoefficientLedger ledger;
  r::NodalRigidPartTopology topology;
  r::NodalRigidPartAssemblyModel parts;
  fe::NodalRigidGroupModel plain;
  fe::NodalRigidAssemblyBinding binding;
  tied::PostKinChkResult post;
  tied::TiedCinAttachmentModel cin_model;
  std::vector<cin::WitnessRange> ranges;
  std::vector<cin::ActiveWitness> witnesses;
  std::vector<double> x,v,w,q,m,j,im,ij;
  std::vector<std::uint8_t> fixed,rotation_fixed,present;
  std::array<std::uint64_t,4> part_ids{777,9302,9303,9304};
  std::array<std::uint64_t,2> plain_ids{10,11};
  std::size_t ordinary = 0,zero_mass = 0;
  explicit Fixture(bool intersection=false);
  fe::NodalStateConfig Config() const;
  fe::HostNodalKinematicsView Kinematics() const {return {x.data(),v.data(),w.data(),m.size(),q.data()};}
  fe::NodalDofConfig Dofs() const {return {fixed.data(),rotation_fixed.data(),ij.data(),present.data()};}
  fe::NodalCinStartup Cin() const {
    return {&cin_model,m.data(),j.data(),ranges.data(),witnesses.data(),witnesses.size(),Qualification};
  }
  void DependentInverses(bool cin);
};
} // namespace rigid_assembly_owner_test
