// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/represented_interval_crossing/DeviceExecution.h"
#include <type_traits>
using Scene = tlfea::contact::represented_interval_crossing::AuthenticatedScene;
using Cohort = tlfea::contact::represented_interval_crossing::AuthenticatedNumericCohort;
static_assert(!std::is_default_constructible_v<Scene>);
static_assert(!std::is_copy_constructible_v<Scene>);
static_assert(!std::is_default_constructible_v<Cohort>);
static_assert(!std::is_copy_constructible_v<Cohort>);

#if defined(TL_TRY_FORGED_SCENE)
void AttemptCallerBracedConstruction() {
  // This compiled before the C++17 passkey fix despite the private key type.
  Scene forged({}, nullptr, nullptr, 0);
}
#elif defined(TL_TRY_FORGED_COHORT)
void AttemptCallerBracedCohort() {
  Cohort forged({}, 0, 1);
}
#else
bool ReadAnExistingBorrow(const Scene& scene) {
  return scene.paths() != nullptr || scene.count() != 0;
}
#endif
