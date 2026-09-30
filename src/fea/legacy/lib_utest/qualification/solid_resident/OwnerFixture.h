// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include "NativeTrajectory.h"
#include "PublicationPeer.h"
#include "lib_src/solvers/ExplicitNodalRigidStep.h"

namespace solid_resident_test {
using Peer=s::BatchQualificationPeer;
inline bool Good(s::BatchReport report) {
  EXPECT_TRUE(report)<<report.message<<" family "<<unsigned(report.family)<<" parent "<<report.parent;
  return bool(report);
}
inline bool Good(fe::NodalReport report) {
  EXPECT_EQ(report.status,fe::NodalStatus::Ok)<<report.message;
  return report.status==fe::NodalStatus::Ok;
}
class SolidResidentCuda:public ::testing::Test {
  void SetUp() override {
    int count=0;
    ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);
    ASSERT_GT(count,0);
  }
};
struct Rig {
  Fixture fixture;
  fe::FENodalState owner;
  s::Batch batch;
  s::BatchConfig config;
  NativeTrajectory native;
  bool Initialize(bool claim=true);
  bool Attach();
  bool Begin(fe::NodalTrialToken&,fe::NodalAssemblyView&);
  bool Prepare(const fe::NodalTrialToken&,const fe::NodalAssemblyView&,fe::NodalPreparedView&);
  bool Read(Results&,s::BatchDiagnostics&);
  void CompareAssembly(const fe::NodalTrialToken&,const fe::NodalAssemblyView&,const Results&);
  bool ComparePrepared(const fe::NodalPreparedView&,const Results&);
};
} // namespace solid_resident_test
