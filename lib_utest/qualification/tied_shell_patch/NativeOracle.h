#pragma once
#include "Fixture.h"

namespace tied_patch_test {
struct NativeResult {
  std::array<double,7> cofactor{};
  std::array<double,24> values{};
};
NativeResult Native(const tie::PatchInput&, const tie::SecondaryLoad&, const tie::MasterMotion&,
                    bool repeated_node = false);
inline void Agreement(const tie::Patch& patch, const tie::MasterLoads& load,
    const tie::SecondaryMotion& motion, const NativeResult& native) {
  for (unsigned i = 0; i < 7; ++i) {
    Near(patch.values().cofactor[i],native.cofactor[i]);
  }
  const auto actual = Values(load,motion);
  ASSERT_EQ(actual.size(),native.values.size());
  for (unsigned i = 0; i < actual.size(); ++i) {
    SCOPED_TRACE(i);
    Near(actual[i],native.values[i]);
  }
}
} // namespace tied_patch_test
