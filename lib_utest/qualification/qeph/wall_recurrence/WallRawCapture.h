#pragma once
#include "MovingNativeProbe.h"
#include "ContactBranchProbe.h"
#include <functional>

namespace tl::qualification::qeph::wall_recurrence {
struct RawStep {
  double h=0;
  std::array<MovingMatrixProbe,3> native;
  std::array<bool,3> native_attempted{};
  std::array<ContactBranchProbe,2> contact;
  std::array<bool,2> contact_attempted{};
};
struct RawJob {
  unsigned cells=0;
  double normal_velocity=0;
  WallRecurrenceModel model;
  std::array<RawStep,6> steps;
  // Complete means all raw matrices and physical samples were collected.
  // It does not imply that derivative, spectral or impact gates passed.
  bool collection_complete=false;
  std::string diagnostic;
};
enum class RawProgressKind { Model,NativeMatrix,ContactBranch,Finished };
struct RawProgress { RawProgressKind kind; unsigned step=0,index=0; };
using RawProgressCallback=std::function<void(const RawJob&,RawProgress)>;

// One fixture/physical boost. Collect every shell matrix first; each contact
// branch then uses the finest native matrix at that h, once per branch.
// Expected numerical rejection retains partials and continues independent
// probes. A callback exception stops work (e.g. exhausted output budget);
// earlier create-only payloads remain durable. No automatic resume protocol.
RawJob CollectRawJob(unsigned cells,double normal_velocity,const RawProgressCallback& callback={});
std::uint64_t PlannedNativeCellIntervals(unsigned cells);
bool CompleteRawStep(const RawStep&,unsigned cells);
} // namespace tl::qualification::qeph::wall_recurrence
