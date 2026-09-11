#pragma once
#include "TestSupport.h"
#include "lib_utest/qualification/solid18_reference/NativeOracle.h"

namespace rear18_test {
struct NativePacket {
  std::array<double,148> values{};
  std::array<int,8> permutation{};
  std::array<double,8> node_mass{};
  int status = -1;
};
NativePacket Native(const s::ReferenceInput& input);
std::array<double,8> Scatter(const law::Reference& reference);
void Compare(const law::Reference& reference, const NativePacket& native);
}  // namespace rear18_test
