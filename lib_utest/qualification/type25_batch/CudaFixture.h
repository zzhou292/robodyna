// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Input.h"
#include "lib_utest/qualification/nodal/NodalTemporalFixture.h"
#include "lib_utest/qualification/type25/EvaluationValues.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "lib_src/elements/type25/Type25BatchIdentity.h"
#include <cstring>

namespace type25_batch_test {
namespace temporal=tl_test::nodal_temporal;
using Type25BatchCuda=temporal::NodalTemporalCuda;
inline void Exact(const spring::Evaluation& a,const spring::Evaluation& b) {
  const auto av=type25_test::EvaluationValues(a),bv=type25_test::EvaluationValues(b);
  for(std::size_t i=0;i<av.size();++i) {SCOPED_TRACE(i);EXPECT_EQ(std::memcmp(&av[i],&bv[i],sizeof(double)),0);}
  EXPECT_EQ(a.history.active,b.history.active);
}
struct Rig {
  Input input;temporal::Initial initial;fe::FENodalState owner;spring::Batch batch;
  bool Initialize(std::size_t count=129,bool moving=false) {
    if(!input.Initialize(count))return false;
    initial.n=5;initial.h=0x1p-26;
    for(std::size_t n=0;n<5;++n) {
      const auto& x=input.shells.nodes()[n].position;initial.x[3*n]=x.x;initial.x[3*n+1]=x.y;initial.x[3*n+2]=x.z;
      if(moving)initial.v[3*n]=8;
      initial.inverse[n]=1/input.mass.nodes()[n].coefficients.mass;
      initial.inverse_inertia[n]=1/input.mass.nodes()[n].coefficients.isotropic_inertia;
    }
    if(initial.Initialize(owner).status!=fe::NodalStatus::Ok)return false;
    auto config=input.Config(owner.accepted());
    if(moving){config.startup.kind=fe::ShellBatchStartupKind::ReferenceUniformTranslation;config.startup.uniform_velocity={8,0,0};}
    const auto created=batch.InitializeJoined(config,input.model,input.mass);EXPECT_EQ(created.status,spring::BatchStatus::Success)<<created.message;
    if(created.status!=spring::BatchStatus::Success)return false;
    fe::NodalTrialToken token;fe::NodalAssemblyView view;
    if(owner.BeginTrial(&token,&view).status!=fe::NodalStatus::Ok)return false;
    const auto bound=batch.AssembleAccepted(owner,view);EXPECT_EQ(bound.status,spring::BatchStatus::Success)<<bound.message;
    owner.Discard();batch.DiscardTrial();return bound.status==spring::BatchStatus::Success;
  }
  bool Prepare(fe::NodalTrialToken& token,fe::NodalPreparedView& prepared) {
    temporal::Loads loads;loads.force[12]=1e5;loads.force[13]=-2e4;loads.couple[14]=3;
    fe::NodalAssemblyView view;
    if(!temporal::BeginLoad(owner,loads,token,view))return false;
    const auto assembled=batch.AssembleAccepted(owner,view);EXPECT_EQ(assembled.status,spring::BatchStatus::Success)<<assembled.message;
    if(assembled.status!=spring::BatchStatus::Success)return false;
    if(owner.SealAssembly(token).status!=fe::NodalStatus::Ok)return false;
    // A test-only single prepared interval under the existing history operation.
    // No independent publication, stability theorem or trajectory is claimed.
    const auto advanced=fe::AdvanceStaggeredHistory(owner,token,{view.owner_id,view.accepted.base_epoch,view.attempt,initial.h,.1,8});
    EXPECT_EQ(advanced.status,fe::NodalStatus::Ok)<<advanced.message;
    return advanced.status==fe::NodalStatus::Ok&&owner.BorrowPrepared(token,&prepared).status==fe::NodalStatus::Ok;
  }
  bool Read(std::vector<spring::Evaluation>& values,spring::BatchDiagnostics& diagnostics) {
    values.resize(input.model.connection_count());const auto copied=batch.CopyAcceptedResults(owner.accepted(),values.data(),values.size(),&diagnostics);
    EXPECT_EQ(copied.status,spring::BatchStatus::Success)<<copied.message;return copied.status==spring::BatchStatus::Success;
  }
  void Discard() {owner.Discard();batch.DiscardTrial();}
};
} // namespace type25_batch_test
