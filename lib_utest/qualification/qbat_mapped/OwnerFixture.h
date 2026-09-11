// SPDX-License-Identifier: MIT
#pragma once
#include "Fixture.h"
#include "lib_src/elements/qbat/QbatBatch.h"
#include "lib_src/elements/qbat/QbatBatchIdentity.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "../qbat_resident/ResultValues.h"

namespace qbat_mapped_test {
inline fe::NodalReport InitializeOwner(Fixture& fixture,fe::FENodalState& owner) {
  const auto cin=fixture.mechanics.Cin();
  return owner.Initialize(fixture.mechanics.Config(),fixture.mechanics.Kinematics(),
      fixture.mechanics.im.data(),fixture.mechanics.Dofs(),fixture.rigid,&cin);
}
struct Rig {
  Fixture fixture;
  fe::FENodalState owner;
  qb::Batch batch;
  qb::BatchConfig config;
  bool Initialize() {
    const auto status=InitializeOwner(fixture,owner);
    EXPECT_EQ(status.status,fe::NodalStatus::Ok)<<status.message;
    if (status.status!=fe::NodalStatus::Ok) return false;
    config=fixture.Config();
    config.owner=owner.accepted();
    const auto result=batch.InitializeMapped(config,fixture.physical,owner,fixture.Witnesses());
    EXPECT_EQ(result.status,qb::BatchStatus::Success)<<result.message;
    return result.status==qb::BatchStatus::Success;
  }
  bool Begin(fe::NodalTrialToken& token,fe::NodalAssemblyView& view) {
    const auto started=owner.BeginTrial(&token,&view);
    EXPECT_EQ(started.status,fe::NodalStatus::Ok);
    if (started.status!=fe::NodalStatus::Ok) return false;
    const auto assembled=batch.AssembleMappedAccepted(owner,token,view);
    EXPECT_EQ(assembled.status,qb::BatchStatus::Success)<<assembled.message;
    return assembled.status==qb::BatchStatus::Success;
  }
  bool Prepare(fe::NodalTrialToken&,fe::NodalAssemblyView&,fe::NodalPreparedView&);
  qb::BatchResult Accepted() {
    qb::BatchResult value;
    qb::BatchDiagnostics diagnostics;
    EXPECT_EQ(batch.CopyAcceptedResults(owner.accepted(),&value,1,&diagnostics).status,qb::BatchStatus::Success);
    return value;
  }
};
} // namespace qbat_mapped_test
