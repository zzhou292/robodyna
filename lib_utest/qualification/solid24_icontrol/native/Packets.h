// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <array>
#include <cstdint>
#include <type_traits>
namespace solid24_icontrol_test::native {
// All dimensional quantities use this one declared working-unit system.
// SI/native conversion belongs at the adapter boundary, never halfway through
// a native routine with its source dimensional floors and tolerances.
enum class WorkingUnits : std::uint32_t { SI=1 };
struct MaterialInput {
  std::array<double,4> law42; // mu,nu,rho,tension_cutoff for the existing native caller.
  // Phase-labelled native PM20,21,22,32,100,107. Produce through authenticated
  // reader/update statements; do not substitute a guessed bulk or modulus.
  std::array<double,6> mechanical_slots;
};
struct State {
  std::array<double,9> material; // stress6,rho,EINT density(index7),bulk pressure.
  std::array<double,12> controlled_hourglass_force; // C:component*4+mode.
  // Native FHOUR(NEL,3,4) atNEL1 usescomponent+3*mode; explicit loops transpose.
  double distortion_energy; // Separate cumulative GBUF%EINT_DISTOR, not EINT density.
};
struct Interval {
  WorkingUnits units;
  std::array<double,24> reference_position,current_position,velocity;
  std::array<double,10> reference_jacobian;
  double reference_volume,base_time,dt;
};
struct StiffnessStages {
  double material_raw,after_controlled_hourglass,after_distortion;
  std::array<double,8> assembled_nodal; // Actual source-ordered SCUMU3 outcome.
};
struct Trial {
  State next;
  std::array<double,24> rhs_force; // Original source-slot order, world coordinates.
  std::array<double,33> material_observation; // Existing independent LAW42 caller ABI.
  StiffnessStages stiffness;
  double hourglass_work_increment,distortion_work_increment,endpoint_time;
  std::uint64_t executed_stages; // Witnesses the requested native branch sequence.
};
inline constexpr std::uint64_t GeometryStage=1,MaterialStage=2,ControlledHourglassStage=4,
    MaterialForceStage=8,WorldRotationStage=16,DistortionParametersStage=32,
    DistortionForceStage=64,NodalAssemblyStage=128;
inline constexpr std::uint64_t RequiredStages=255;
static_assert(std::is_standard_layout_v<State> && std::is_trivially_copyable_v<State>);
static_assert(sizeof(State)==22*sizeof(double));
// Internal qualification contract. NativeSupport.h owns the explicit flat-wire
// adapter; no production solver API or physical runtime is enabled.
inline constexpr bool StrainOutputHistoryQualified=false; // ANIM_N=0, no STRHG18.
} // namespace solid24_icontrol_test::native
