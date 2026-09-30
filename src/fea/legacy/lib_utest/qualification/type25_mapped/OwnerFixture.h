// SPDX-License-Identifier: MIT
#pragma once
#include "Fixture.h"
#include "NativeValues.h"
#include "PublicationPeer.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace type25_mapped_test {
inline fe::NodalReport InitializeOwner(Fixture& f,fe::FENodalState& owner) {
  const auto cin=f.base.mechanics.Cin();
  return owner.Initialize(f.base.mechanics.Config(),f.base.mechanics.Kinematics(),
      f.base.mechanics.im.data(),f.base.mechanics.Dofs(),f.base.rigid,&cin);
}
struct Rig {
  Fixture fixture;
  fe::FENodalState owner;
  spring::Batch batch;
  spring::BatchConfig config;
  explicit Rig(bool failing=false):fixture(failing) {}
  bool Initialize();
  bool Begin(fe::NodalTrialToken&,fe::NodalAssemblyView&);
  bool Prepare(fe::NodalTrialToken&,fe::NodalAssemblyView&,fe::NodalPreparedView&);
  std::vector<spring::Evaluation> Accepted();
  std::vector<spring::EndpointKinematics> Nodes(const fe::NodalPreparedView&);
  std::vector<double> Snapshot();
  void CheckAssembly(const fe::NodalTrialToken&,const fe::NodalAssemblyView&);
};
}
