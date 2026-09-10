#pragma once
#include "QephCoupledFixture.h"

namespace qeph_coupled_test {
struct LedgerEvidence {
  double kick_ratio=0,momentum_ratio=0,angular_ratio=0,internal_work_ratio=0;
  double source_work_ratio=0;
  double angular_drift_rounding=0,source_internal_work=0;
};
struct LedgerScales {
  long double energy=0,linear=0,angular=0;
};
// Long-double accounting of actual stored binary64 inputs, not another force
// or constitutive equation. Physical floors are frozen in the design document.
void CheckLedgers(const Rig&,const Snapshot& base,const Prepared&,const Loads& applied,
                  const PortResults& accepted,const q::BatchDiagnostics&,LedgerEvidence&);
void CheckLedgers(const Rig&,const Snapshot& base,const Prepared&,const Loads& applied,
                  const PortResults& accepted,const q::BatchDiagnostics&,const LedgerScales&,LedgerEvidence&);
// Check all accumulated and interval source-work aggregates from the candidate
// element histories and diagnostics, independently of the batch reduction.
void CheckSourceWork(const Rig&,const PortResults& accepted,const PortResults& candidate,
                     const q::BatchDiagnostics&,LedgerEvidence&);
void CheckSourceWork(const Rig&,const PortResults& accepted,const PortResults& candidate,
                     const q::BatchDiagnostics&,long double energy_scale,LedgerEvidence&);
double WrongKickSeparation(const Rig&,const Snapshot& base,const Prepared&,const Loads& applied,bool initial);
} // namespace qeph_coupled_test
