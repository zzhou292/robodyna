// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "FailureKey.h"
#include "ScreenSummary.h"
#include "GroupReport.h"
#include "../../constraints/tied_shell/runtime/CinForceTransfer.h"
#include "../FENodalStateStorage.h"
#include "../NodalCinRuntime.h"
#include "../../constraints/NodalRigidGroupState.h"
#include "../../constraints/NodalRigidGroupStepMath.h"
#include "../../constraints/NodalRigidAccelerationSink.h"
namespace tl::fea::cin_advance {
// Private launch views into the one owner's existing accepted/trial/work slabs.
// They provide no public admission, lifetime, selector or publication authority.
struct Input {
  nodal_detail::Control* control = nullptr;
  const double* accepted = nullptr;
  double* trial = nullptr;
  double* loads = nullptr;
  const std::uint8_t* fixed = nullptr;
  constraints::tied_shell::cin::StageView model;
  double* tail = nullptr;
  double* work = nullptr;
  constraints::tied_shell::Patch* patches = nullptr;
  const std::uint8_t* activity = nullptr;
  rigid::GroupDeviceView groups;
  rigid::StepDurations durations;
  double maximum_angle = 0;
  std::uint64_t epoch = 0, attempt = 0;
  rigid::AccelerationSink capture;
  const std::uint8_t* rotation_present = nullptr;
  NodalCinStructuralStep structural;
  FailureKey* failure = nullptr;
  // Separate input scan key; null retains the private serial-prefix fixture path.
  FailureKey* input_failure = nullptr;
  // Count is Blocks(model.node_count); null preserves the private serial screen.
  screen::Summary* screen = nullptr;
  // Exactly groups.group_count records; null retains the frozen serial suffix.
  groups::Report* group_reports = nullptr;
  // Exactly model.row_count packets, rewritten after complete input/entry-IN
  // preparation each attempt. Null retains the public serial force route.
  constraints::tied_shell::cin::detail::PreparedForceRow* prepared_transfers = nullptr;
};
cudaError_t Launch(const Input&, cudaStream_t);
} // namespace tl::fea::cin_advance
