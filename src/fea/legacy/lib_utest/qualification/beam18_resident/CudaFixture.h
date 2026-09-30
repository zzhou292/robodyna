// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "OwnerFixture.h"
#include "PublicationPeer.h"
#include "lib_utest/qualification/beam18_force/NativeOracle.h"
#include <cuda_runtime.h>
namespace beam18_resident_test {
using Peer = b::BatchQualificationPeer;
using Results = std::vector<b::Result>;
inline b::ResultBuffer Buffer(Results& r) { return {r.data(), r.size()}; }
inline bool Good(b::BatchReport r) { EXPECT_TRUE(r) << r.message; return bool(r); }
inline bool Good(fe::NodalReport r) { EXPECT_EQ(r.status, fe::NodalStatus::Ok) << r.message; return r.status == fe::NodalStatus::Ok; }
struct Rig {
  OwnerFixture fixture;
  fe::FENodalState owner;
  b::Batch batch;
  b::BatchConfig config;
  std::vector<beam18_force_test::NativeResult> native_accepted, native_candidate;
  bool Initialize(bool attach = true);
  bool Attach();
  bool Begin(fe::NodalTrialToken&, fe::NodalAssemblyView&);
  bool Prepare(const fe::NodalTrialToken&, const fe::NodalAssemblyView&, fe::NodalPreparedView&);
  bool Compare(const Results&, bool initial, const fe::NodalPreparedView* = nullptr);
  bool Read(Results&, b::BatchDiagnostics&);
};
void CompareResult(const b::Parent&, const b::Material&, const b::Result&, const beam18_force_test::NativeResult&);
void SameResults(const Results&, const Results&);
void CheckAssembly(Rig&, const fe::NodalTrialToken&, const fe::NodalAssemblyView&, const Results&);
} // namespace beam18_resident_test
