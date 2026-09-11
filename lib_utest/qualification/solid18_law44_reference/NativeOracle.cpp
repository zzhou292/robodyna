#include "NativeOracle.h"
#include <gtest/gtest.h>
extern "C" void rear18_reference_native(const double*, const double*, const int*, double*, int*, double*, int*);

namespace rear18_test {
namespace {
std::array<int,8> Local(const s::ReferenceInput& input) {
  std::array<int,8> result{};
  int count = 0;
  for (unsigned n = 0; n < 8; ++n) {
    unsigned prior = 0;
    while (prior < n && input.source_node_id[prior] != input.source_node_id[n]) ++prior;
    result[n] = prior < n ? result[prior] : count++;
  }
  return result;
}
}
NativePacket Native(const s::ReferenceInput& input) {
  double x[24];
  for (unsigned n = 0; n < 8; ++n) {
    x[3*n] = input.position_m[n].x;
    x[3*n+1] = input.position_m[n].y;
    x[3*n+2] = input.position_m[n].z;
  }
  const auto local = Local(input);
  NativePacket result;
  rear18_reference_native(x, &input.density_kg_m3, local.data(), result.values.data(),
                          result.permutation.data(), result.node_mass.data(), &result.status);
  return result;
}
std::array<double,8> Scatter(const law::Reference& reference) {
  const auto local = Local(reference.input());
  std::array<double,8> result{};
  // Native slot order is preserved even after initial orientation reversal.
  for (unsigned n = 0; n < 8; ++n) {
    const auto source = reference.source_slot(n);
    result[local[source]] += reference.mass().source_nodal_mass_kg[source];
  }
  return result;
}
void Compare(const law::Reference& reference, const NativePacket& native) {
  ASSERT_EQ(native.status, 0);
  EXPECT_TRUE(solid18_test::Agree(Values(reference), native.values));
  const auto mass = Scatter(reference);
  for (unsigned n = 0; n < 8; ++n) {
    EXPECT_EQ(reference.source_slot(n), static_cast<unsigned>(native.permutation[n]));
    EXPECT_TRUE(solid18_test::Close(mass[n], native.node_mass[n]));
    EXPECT_EQ(mass[n] == 0, native.node_mass[n] == 0);
  }
}
}  // namespace rear18_test
