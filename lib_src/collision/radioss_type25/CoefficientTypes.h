// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
namespace tlfea::contact::radioss_type25 {
enum class CoefficientStatus { Ok, InvalidInput, UnsupportedProfile, NonfiniteResult };
enum class MainFaceKind { Unspecified, OrdinaryExterior, Coating, Internal };
enum class ShellLayout { Unspecified, Quad4, Triangle3 };
enum class SolidLayout { Unspecified, EightSlot, TenNode, TwentyNode, SixteenNode };
// These are arithmetic operands, not source-binding or incidence authority.
template<class U> struct ShellMainCoefficientInput {
  MainFaceKind face = MainFaceKind::Unspecified;
  ShellLayout layout = ShellLayout::Unspecified;
  int property_type = 0;       // IGTYP = IGEO(11,MG).
  int stack_material = 0;      // Resolved IGMAT branch operand; see source-binding caveat.
  int input_thickness_mode = 0; // Native IINTTHICK: zero admits element override.
  double scale = 1;
  double element_thickness = 0, property_thickness = 0;
  double young = 0; // Resolved PM20, not plane-stress PM24 or current tangent.
};
template<class U> struct SolidMainCoefficientInput {
  MainFaceKind face = MainFaceKind::Unspecified;
  SolidLayout layout = SolidLayout::Unspecified;
  int incompressibility_control = 0; // ICONTR: exactly 1 selects PM107.
  double scale = 1, fill = 1;
  double area = 0, volume = 0; // Authentic INSOL3D/VOLINT outputs.
  double bulk = 0, controlled_bulk = 0; // PM32 and PM107.
};
template<class U> struct SolidMainCoefficientResult {
  double stiffness = 0;
  double characteristic_length = 0; // Native GAP_N(1), NOT the contact gap GAP_M.
};
template<class U> struct ScalarCoefficient { double value = 0; };
// Accumulated over the WHOLE physical model before ASSTIFI. Existing stiffness
// may already contain beam/other contributions. This leaf performs no gathering.
template<class U> struct AccumulatedNodalCoefficients {
  double volume = 0;             // VOLNOD, volume.
  double bulk_volume = 0;        // BVOLNOD before normalization, pressure*volume.
  double young_thickness_sum = 0; // ETNOD, stiffness.
  int shell_incidence_count = 0; // NSHNOD, original integer incidence count.
  double existing_stiffness = 0; // STIFINT before ASSTIFI.
};
template<class U> struct NodalCoefficientResult {
  double normalized_bulk = 0; // BVOLNOD AFTER normalization, pressure.
  double stiffness = 0;
};
template<class U> struct SecondaryCoefficientInput {
  double existing = 0; // Existing interface STFNS; exact zero is a removal mask.
  double global_stiffness = 0; // Global STIFINT after all required corrections.
  double scale = 1; // I25STSECND uses 1 when scale is not positive.
};
struct PairCoefficientProfile {
  int stiffness_formulation = -1; // Only observed IGSTI4 is admitted.
  int mass_timestep_augmentation = -1; // Observed ISTIF_MSDT=0, explicitly resolved.
};
template<class U> struct PairCoefficientInput {
  double main = 0, secondary = 0;
  double minimum = 0, maximum = 0; // Native KMIN/KMAX in this packet's units.
};
using NativeShellMainCoefficientInput = ShellMainCoefficientInput<NativeUnitsTag>;
using SiShellMainCoefficientInput = ShellMainCoefficientInput<SiUnitsTag>;
using NativeSolidMainCoefficientInput = SolidMainCoefficientInput<NativeUnitsTag>;
using SiSolidMainCoefficientInput = SolidMainCoefficientInput<SiUnitsTag>;
using NativeSolidMainCoefficientResult = SolidMainCoefficientResult<NativeUnitsTag>;
using SiSolidMainCoefficientResult = SolidMainCoefficientResult<SiUnitsTag>;
using NativeScalarCoefficient = ScalarCoefficient<NativeUnitsTag>;
using SiScalarCoefficient = ScalarCoefficient<SiUnitsTag>;
using NativeAccumulatedNodalCoefficients = AccumulatedNodalCoefficients<NativeUnitsTag>;
using SiAccumulatedNodalCoefficients = AccumulatedNodalCoefficients<SiUnitsTag>;
using NativeNodalCoefficientResult = NodalCoefficientResult<NativeUnitsTag>;
using SiNodalCoefficientResult = NodalCoefficientResult<SiUnitsTag>;
using NativeSecondaryCoefficientInput = SecondaryCoefficientInput<NativeUnitsTag>;
using SiSecondaryCoefficientInput = SecondaryCoefficientInput<SiUnitsTag>;
using NativePairCoefficientInput = PairCoefficientInput<NativeUnitsTag>;
using SiPairCoefficientInput = PairCoefficientInput<SiUnitsTag>;
} // namespace tlfea::contact::radioss_type25
