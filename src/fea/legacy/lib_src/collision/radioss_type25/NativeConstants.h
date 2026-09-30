// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss constant_mod.F, a62b27e6, MYREAL8. Native working-unit constants.
#pragma once
namespace tlfea::contact::radioss_type25::native_constant {
inline constexpr double ep10 = 1e10;
inline constexpr double ep20 = 1e20;
inline constexpr double epp = 1. / ep10;
inline constexpr double em20 = 1. / ep20;
inline constexpr double em30 = 1. / (ep20 * ep10);
} // namespace tlfea::contact::radioss_type25::native_constant
