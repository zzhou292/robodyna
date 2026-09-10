#pragma once
#include "../wall_coupled/WallCoupledFixture.h"
#include "../wall_recurrence/WallRecurrenceModel.h"
#include "../wall_recurrence/WallSwitchingSchedule.h"
#include <string>

namespace qeph_wall_incoming_test {
using namespace qeph_wall_test;
namespace wr=tl::qualification::qeph::wall_recurrence;
constexpr std::uint64_t IncomingQualification=0x435732454e545231ULL;
constexpr std::uint64_t IncomingShellConfiguration=0x4357325348454c31ULL;
constexpr std::uint64_t IncomingWallConfiguration=0x43573257414c4c31ULL;
constexpr std::uint64_t IncomingWallBinding=0x43573246494e4931ULL;
constexpr double PrefixHorizon=224*H0;
constexpr unsigned MaximumPrefixSteps=896;
struct ScreenBinding {
  double selected_h=0;
  std::string decision_sha256;
};
// Root authenticates the actual decision/source/config before launch. The
// SHA256 is the final selection index.json (binding selection.json, all four
// boost comparisons and provenance). Format checking does not create authority.
// No default H0, solver-side selection, parser duplication or skipped GPU test.
bool ReadScreenBinding(ScreenBinding&,std::string&);
bool ValidScreenBinding(const ScreenBinding&);
struct IncomingRig {
  const ScreenBinding binding;
  const wr::WallRecurrenceModel model;
  WallRig coupled;
  std::array<double,3*N> velocity{};
  LedgerScales scales;
  wr::WallSwitchingSchedule schedule;
  unsigned intervals=0;
  bool initialization_attempted=false;
  explicit IncomingRig(unsigned cells,const ScreenBinding&);
  bool Initialize(unsigned refinement);
  bool InitializeNative(NativeSequence&) const;
};
struct EntryObservation {
  std::uint64_t base_epoch=0;
  double base_time=0,endpoint_time=0;
  unsigned base_mask=0,endpoint_mask=0;
  double minimum_base_gap=0,maximum_base_gap=0,minimum_endpoint_gap=0,maximum_endpoint_gap=0;
  double minimum_velocity=0,maximum_velocity=0;
  bool crossing=false;
};
struct IncomingEvidence {
  LedgerEvidence source;
  ContactEvidence contact;
  std::array<EntryObservation,MaximumPrefixSteps> accepted{};
  unsigned count=0;
  std::uint64_t crossing_base=UINT64_MAX,first_applied_base=UINT64_MAX;
  double omitted_contact=0,premature_endpoint=0;
  double omitted_signal=0,omitted_budget=0,premature_signal=0,premature_budget=0;
  double maximum_regular_rate_ratio=0,maximum_hourglass_rate_ratio=0;
};
struct IncomingStage {
  Staged coupled;
  EntryObservation event;
  LedgerEvidence source;
  ContactEvidence contact;
  double omitted_contact=0,premature_endpoint=0;
  double omitted_signal=0,omitted_budget=0,premature_signal=0,premature_budget=0;
  double maximum_regular_rate_ratio=0,maximum_hourglass_rate_ratio=0;
  bool checked=false;
};
void CheckInitial(const IncomingRig&,const Snapshot&,const PortResults&,const q::BatchDiagnostics&);
bool CheckIncoming(const IncomingRig&,const Snapshot&,const Trial&,const Staged&,const PortResults&,
                   const NativeProposal&,const IncomingEvidence&,IncomingStage&);
bool PublishIncoming(IncomingRig&,const Trial&,const IncomingStage&,sc::NodalWallDeviceResults&,IncomingEvidence&);
void CheckFinished(const IncomingRig&,const IncomingEvidence&);
// One complete native/device interval, including all gates before its receipt.
void AdvanceIncoming(IncomingRig&,NativeSequence&,IncomingEvidence&,sc::NodalWallDeviceResults&);
void RecordIncoming(const IncomingRig&,const IncomingEvidence&,const std::string& suffix);
void IncomingPrefix(unsigned cells,const ScreenBinding&);
void IncomingFailures(const ScreenBinding&);
} // namespace qeph_wall_incoming_test
