#pragma once
#include "WallStateMetric.h"
#include <array>
#include <vector>

namespace tl::qualification::qeph::wall_recurrence {
// Finite signed gap from the wall. Match the owning point's touching-or-
// penetrating mask, including zero force at exact touching (also signed zero).
constexpr bool WallScalarContactActive(double position) noexcept { return position>=0; }
struct ScalarContactState {
  double position=0,velocity=0;
  bool active=false; // This state's base gap controls its outgoing kick.
};
struct WallSwitchingWindow {
  int entry_shift=0,exit_shift=0;
  unsigned entry_base_epoch=0,exit_base_epoch=0;
  unsigned inactive_before=0,active=0,inactive_after=0;
};
struct WallSwitchingSchedule {
  double h=0,frequency=0,maximum_depth=0;
  unsigned total_steps=0,ordinary_steps=0,entry_base_epoch=0,exit_base_epoch=0;
  std::vector<ScalarContactState> scalar; // Epochs 0..total_steps inclusive.
  std::array<WallSwitchingWindow,9> windows{}; // Entry-major {-1,0,+1}².
  bool complete=false,passed=false;
  std::string diagnostic;
};
// Generic index seam for direct chronological/count tests; all nine windows
// must fit before any output is published. Products start at base epoch one.
bool BuildWallSwitchingWindows(unsigned total_steps,unsigned entry,unsigned exit,
                              std::array<WallSwitchingWindow,9>&,std::string&);
// Independent prospective scalar schedule, not a mechanics owner. Its first
// kick is known zero; thereafter each base gap chooses the ordinary full kick.
bool BuildWallSwitchingSchedule(const WallRecurrenceModel&,double h,
                               WallSwitchingSchedule&,std::string&);
// Evidence form retains the scalar trace if event-window validation fails.
WallSwitchingSchedule AnalyzeWallSwitchingSchedule(const WallRecurrenceModel&,double h);
} // namespace tl::qualification::qeph::wall_recurrence
