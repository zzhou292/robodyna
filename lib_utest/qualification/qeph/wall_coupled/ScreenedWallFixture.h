#pragma once
// Shared qualification-only preparation/checks extracted from retained CW2.
// No new owner, force law, clock, receipt or choice of experiment/timestep.
#include "WallCoupledFixture.h"
#include "../wall_recurrence/WallRecurrenceModel.h"

namespace qeph_wall_test {
namespace screened_wall=tl::qualification::qeph::wall_recurrence;
using ScreenedVelocity=std::array<double,3*N>;
// Caller fixes the immutable experiment IDs/law/startup and validates its h,
// horizon and one-time initialization before this existing preparation recipe.
// Native mass/J and the ledger scales are prepared once, then read immutably.
bool InitializeScreenedWall(WallRig&,const screened_wall::WallRecurrenceModel&,
                            ScreenedVelocity&,LedgerScales&);
bool InitializeScreenedNative(const WallRig&,const ScreenedVelocity&,NativeSequence&);
void CheckScreenedInitial(const WallRig&,const LedgerScales&,const Snapshot&,
                          const PortResults&,const q::BatchDiagnostics&);
void CheckScreenedRigid(const WallRig&,const Trial&,const Staged&,const LedgerScales&,
                        double& regular_maximum,double& hourglass_maximum);
} // namespace qeph_wall_test
