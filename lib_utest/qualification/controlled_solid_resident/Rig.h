// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../extended_solid_resident/OwnerFixture.h"
#include "lib_src/elements/solid24/controlled_distortion/Stage.h"
#include "lib_src/elements/solid18/total_strain/controlled_distortion/Stage.h"
#include "lib_src/solvers/ExplicitNodalRigidStep.h"
namespace controlled_resident_test {
using namespace extended_resident_test;
using Peer=s::BatchQualificationPeer;
namespace h24=fe::solid24::controlled_distortion;
namespace foam=fe::solid18::total_strain::controlled_distortion;
inline bool Good(s::BatchReport r){EXPECT_TRUE(r)<<r.message<<" family "<<unsigned(r.family)<<" parent "<<r.parent;return bool(r);}
inline bool Good(fe::NodalReport r){EXPECT_EQ(r.status,fe::NodalStatus::Ok)<<r.message;return r.status==fe::NodalStatus::Ok;}
struct Results {
  std::vector<s::Result18> old18;
  std::vector<s::ProfiledResult24> h24;
  std::vector<s::Result6z> old6z;
  std::vector<s::Result18Law44> rear;
  std::vector<s::ProfiledResult18Law90> foam;
  explicit Results(const s::Model& m):old18(m.solid18().size()),h24(m.solid24().size()),old6z(m.solid6z().size()),rear(m.solid18_law44().size()),foam(m.solid18_law90().size()){}
  s::ProfiledResultBuffers Buffers(){return {old18.data(),old18.size(),h24.data(),h24.size(),old6z.data(),old6z.size(),rear.data(),rear.size(),foam.data(),foam.size()};}
};
struct Rig {
  OwnerFixture fixture;
  fe::FENodalState owner;s::Batch batch;s::BatchConfig config;
  h24::Reference hreference;foam::Reference freference;
  h24::Result hexpected;foam::Result fexpected;
  explicit Rig(s::control::UnitScale units={1,1,1},bool collapsed=false):fixture(false,true,units,collapsed){}
  bool Initialize();bool Read(Results&,s::BatchDiagnostics&);
  bool Begin(fe::NodalTrialToken&,fe::NodalAssemblyView&);
  bool Prepare(const fe::NodalTrialToken&,const fe::NodalAssemblyView&,fe::NodalPreparedView&);
  bool ReferenceStep(const fe::NodalPreparedView&);
  void Compare(const Results&);
  void CheckAssembly(const fe::NodalTrialToken&,const fe::NodalAssemblyView&,const Results&);
};
class ControlledResidentCuda:public ::testing::Test {void SetUp()override{int n=0;ASSERT_EQ(cudaGetDeviceCount(&n),cudaSuccess);ASSERT_GT(n,0);}};
} // namespace controlled_resident_test
