// SPDX-License-Identifier: MIT
#pragma once
#include "Fixture.h"
#include "../qbat_force/NativeOracle.h"
#include "lib_src/elements/ShellBatchPublication.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"
#include "lib_src/assembly/NodalMassBinding.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include <cuda_runtime.h>
#include <vector>

namespace qbat_resident_test {
using Cuda=::testing::Test;
inline constexpr std::uint64_t Configuration=0x5142415452455349ULL,Qualification=0x514241544e415449ULL;
struct Fields {
  std::vector<double> x,v,omega,q;
  fe::NodalStamp stamp;
  explicit Fields(std::size_t n=0):x(3*n),v(3*n),omega(3*n),q(4*n) {}
};
struct Prepared {
  fe::NodalTrialToken token;
  fe::NodalPreparedView view;
  Fields endpoint;
  fe::ShellBatchDiagnostics diagnostics;
  std::vector<qb::BatchResult> qbat;
};
struct Rig {
  const fe::ShellBatchBinding* binding=nullptr;
  const fe::NodalMassBinding* mass=nullptr;
  fe::FENodalState owner;
  fe::qeph::QephBatch qeph;
  fe::t3::T3Batch t3;
  qb::Batch qbat;
  fe::type25::Batch connector;
  fe::ShellBatchPublication publication;
  double* loads=nullptr;
  double dt=0x1p-20;
  bool coupled=false;
  ~Rig();
  bool Initialize(const fe::ShellFormulationScope&,bool coupled=false,bool moving=false,double h=0x1p-20,
      const fe::type25::Model* connector_model=nullptr);
  bool Assemble(fe::NodalAssemblyView);
  bool Prepare(Prepared&,double target_rate=0,bool evaluate=true);
  bool Evaluate(Prepared&);
  bool Commit(const Prepared&);
  bool Accepted(Fields&,std::vector<qb::BatchResult>&,fe::ShellBatchDiagnostics&);
  void Discard();
};
bool Endpoint(const fe::NodalPreparedView&,Fields&);
qb::PrescribedInterval Interval(const Rig&,const Prepared&,std::size_t parent);
qb::ForceTrial Restore(const batch::Element&,const qb::BatchResult&);
void Exact(const std::vector<qb::BatchResult>&,const std::vector<qb::BatchResult>&);
void SameFields(const Fields&,const Fields&);
enum class ReadFault { None, LateNonfinite, InvalidFlag, CopyError };
void Arm(ReadFault,std::size_t parents);
} // namespace qbat_resident_test
