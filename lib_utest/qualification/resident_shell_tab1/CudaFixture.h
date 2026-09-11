#pragma once
#include "Values.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"
#include "lib_src/elements/ShellBatchPublication.h"
#include "lib_utest/qualification/nodal/NodalTemporalFixture.h"

namespace resident_tab1_test {
namespace temporal = tl_test::nodal_temporal;
namespace q = fe::qeph;
namespace t = fe::t3;
constexpr double H = failure_force_test::Dt;
constexpr std::uint64_t Configuration = 0x52544231434f4c31ULL, Qualification = 0x5254423148415231ULL;
using Cuda = temporal::NodalTemporalCuda;
struct Frame {
  std::array<q::ForceTrial, Parents> qforce;
  std::array<t::ForceTrial, Parents> tforce;
  std::array<fe::ShellBatchLayeredSection, Parents> qsection, tsection;
  std::array<fe::ShellBatchFailureState, Parents> qfailure, tfailure;
  fe::ShellBatchDiagnostics diagnostics;
};
struct Prepared {
  fe::NodalTrialToken token;
  fe::NodalPreparedView view;
  temporal::Snapshot endpoint;
};
struct Rig {
  fe::ShellBatchBinding binding;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  temporal::Initial initial;
  fe::FENodalState owner;
  q::QephBatch qeph;
  t::T3Batch t3;
  fe::ShellBatchPublication publication;
  bool Initialize(Placement, bool sibling_mismatch = false, bool omit_failure = false);
  bool Bind();
  void Discard() { owner.Discard(); publication.DiscardTrial(); qeph.DiscardTrial(); t3.DiscardTrial(); }
};
bool Prepare(Rig&, Prepared&);
bool Read(Rig&, Frame&, const fe::ShellBatchDiagnostics* = nullptr);
bool Evaluate(Rig&, const Prepared&, Frame&);
bool Commit(Rig&, const Prepared&, const Frame&);
void Same(const Frame&, const Frame&);
q::PrescribedInterval Interval(const q::ReferenceData&, const std::array<std::size_t, 4>&, const Prepared&);
t::PrescribedInterval Interval(const t::ReferenceData&, const std::array<std::size_t, 3>&, const Prepared&);
enum class ReadFault { None, NonfiniteDamage, InvalidDisplayCap, InvalidPolicy, InvalidFlag };
void Arm(ReadFault);
unsigned Copies();
} // namespace resident_tab1_test
