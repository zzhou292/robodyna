// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../FrictionTypes.h"
#include <cstddef>
#include <cstdint>
namespace tlfea::contact::radioss_type25::assembly {
enum class Status { Ok, InvalidInput, UnsupportedProfile, NonfiniteResult };
struct Controls {
  int parallel_assembly = -1, pinch = -1, thermal = -1;
  int thermal_formulation = -1, thermal_nodal_timestep = -1;
  EngineControls engine;
};
// This profile has local endpoints only. Native negative NSVG addresses a
// separate remote buffer; it must never be converted to a local-node alias.
struct Connectivity {
  std::uint32_t main[4]{UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
  std::uint32_t secondary = UINT32_MAX;
};
template<class Units> struct Endpoints {
  Vector main_force[4]{};
  double main_stiffness[4]{};
  Vector secondary_resultant{}; // Subtracted, not pre-negated then added.
  double secondary_stiffness = 0;
  bool active = false; // HH != 0, using the post-response weights.
};
template<class Units> struct NodalValue {
  Vector force{};
  double stiffness = 0;
};
using NativeEndpoints = Endpoints<NativeUnitsTag>;
using SiEndpoints = Endpoints<SiUnitsTag>;
using NativeNodalValue = NodalValue<NativeUnitsTag>;
using SiNodalValue = NodalValue<SiUnitsTag>;
// Exclusive cumulative row ends partition the original logical ASS0 calls.
// Per cohort: all rows' four main slots, then all rows' secondary slots.
// They are numerical ordering metadata, not GPU launch sizes.
struct Schedule {
  const std::uint32_t* cohort_ends = nullptr;
  std::size_t cohort_count = 0, row_count = 0;
};
struct Incidence {
  const std::uint32_t* offsets = nullptr;
  const std::uint32_t* occurrences = nullptr;
  std::size_t node_count = 0, occurrence_count = 0;
};
struct Occurrence { std::uint32_t row = 0, slot = 0; }; // slot4 is secondary.
} // namespace tlfea::contact::radioss_type25::assembly
