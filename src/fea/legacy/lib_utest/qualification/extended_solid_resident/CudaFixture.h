// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeChecks.h"
#include "lib_src/solvers/ExplicitNodalRigidStep.h"
namespace extended_resident_test {
using Peer=s::BatchQualificationPeer;
inline bool Good(s::BatchReport r) { EXPECT_TRUE(r)<<r.message<<" family "<<unsigned(r.family)<<" parent "<<r.parent;return bool(r); }
inline bool Good(fe::NodalReport r) { EXPECT_EQ(r.status,fe::NodalStatus::Ok)<<r.message;return r.status==fe::NodalStatus::Ok; }
struct Rig {
  OwnerFixture fixture;
  fe::FENodalState owner;
  s::Batch batch;
  s::BatchConfig config;
  NativeChecks native;
  bool Initialize(bool attach=true);
  bool Attach();
  bool Read(Results&,s::BatchDiagnostics&);
  bool Begin(fe::NodalTrialToken&,fe::NodalAssemblyView&);
  bool Prepare(const fe::NodalTrialToken&,const fe::NodalAssemblyView&,fe::NodalPreparedView&);
  bool Compare(const fe::NodalPreparedView&,const Results&);
  void CheckAssembly(const fe::NodalTrialToken&,const fe::NodalAssemblyView&,const Results&,double stiffness_seed=0);
};
class ExtendedResidentCuda:public ::testing::Test {
  void SetUp() override {int n=0;ASSERT_EQ(cudaGetDeviceCount(&n),cudaSuccess);ASSERT_GT(n,0);}
};
} // namespace extended_resident_test
