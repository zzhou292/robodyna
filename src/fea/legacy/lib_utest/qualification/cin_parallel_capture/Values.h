// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "FrozenCopy.h"
#include "lib_src/solvers/cin_advance/Capture.h"
#include <gtest/gtest.h>
#include <cstring>
#include <vector>

namespace tl::fea::cin_capture_test {
struct Values {
  std::uint32_t nodes;
  std::vector<double> work, output;
  std::vector<std::uint8_t> members;
  nodal_detail::Control control;
  explicit Values(std::uint32_t n) : nodes(n), work(9*n), output(6*n, -91), members(n) {
    constexpr std::uint64_t bits[]{0, 0x8000000000000000ULL, 1, 0x8000000000000001ULL,
      0x0010000000000000ULL, 0x7ff0000000000000ULL, 0xfff0000000000000ULL,
      0x7ff8000000001357ULL, 0x3ff0000000000001ULL};
    for (std::size_t i = 0; i < work.size(); ++i) {
      const auto value = bits[i%9];
      std::memcpy(&work[i], &value, sizeof(value));
    }
    for (std::uint32_t i = 0; i < n; ++i) members[i] = i%5 == 0 ? std::uint8_t(1+i%255) : 0;
  }
  cin_advance::Input Input() {
    cin_advance::Input input;
    input.control = &control;
    input.model.node_count = nodes;
    input.work = work.data();
    input.groups.member_nodes = members.data();
    input.capture.node = output.data();
    input.capture.node_rotation = output.data()+3*nodes;
    return input;
  }
};
inline void SameValues(const std::vector<double>& a, const std::vector<double>& b) {
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    ASSERT_EQ(std::memcmp(&a[i], &b[i], sizeof(double)), 0) << "slot " << i;
  }
}
} // namespace tl::fea::cin_capture_test
