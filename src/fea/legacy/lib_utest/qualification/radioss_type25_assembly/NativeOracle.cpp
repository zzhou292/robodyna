// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <array>
#include <climits>
#include <stdexcept>
extern "C" void rd_assembly(int, int, int*, int*, int*, int*, int*,
    double*, double*, double*, double*, double*, double*, double*, double*,
    double*, double*, double*);
namespace type25_assembly_test {
std::vector<ass::NativeNodalValue> NativeAssemble(
    const std::vector<ass::Connectivity>& rows,
    const std::vector<rd::NativeFrictionResult>& responses,
    const std::vector<std::uint32_t>& cohort_ends,
    const std::vector<ass::NativeNodalValue>& incoming) {
  if (rows.size() != responses.size() || rows.size() > INT_MAX ||
      incoming.empty() || incoming.size() > INT_MAX ||
      (rows.empty() != cohort_ends.empty()) ||
      (!rows.empty() && cohort_ends.back() != rows.size()))
    throw std::invalid_argument("Native local ASS0 dimensions");
  std::vector<double> a(3 * incoming.size()), stiffness(incoming.size());
  for (std::size_t n = 0; n < incoming.size(); ++n) {
    a[3*n] = incoming[n].force.x; a[3*n+1] = incoming[n].force.y;
    a[3*n+2] = incoming[n].force.z; stiffness[n] = incoming[n].stiffness;
  }
  std::uint32_t first = 0;
  for (auto end : cohort_ends) {
    if (end <= first || end > rows.size()) throw std::invalid_argument("Native ASS0 cohort");
    const auto count = end - first;
    std::array<std::vector<int>,5> node;
    std::array<std::vector<double>,9> value;
    for (auto& v : node) v.resize(count);
    for (auto& v : value) v.resize(count);
    for (std::uint32_t i = 0; i < count; ++i) {
      const auto& response = responses[first+i];
      for (unsigned slot = 0; slot < 5; ++slot) {
        const auto at = slot < 4 ? rows[first+i].main[slot] : rows[first+i].secondary;
        if (at >= incoming.size()) throw std::invalid_argument("Native ASS0 nonlocal node");
        node[slot][i] = static_cast<int>(at + 1);
        if (slot < 4) value[slot][i] = response.normal.weights[slot];
      }
      if (!response.contact_active) {
        for (unsigned slot = 0; slot < 4; ++slot)
          if (response.normal.weights[slot] != 0)
            throw std::invalid_argument("Native inactive response weights were not cleared");
      }
      value[4][i] = response.normal.stability_stiffness;
      value[5][i] = response.native_resultant.x;
      value[6][i] = response.native_resultant.y;
      value[7][i] = response.native_resultant.z;
      // The selected endpoint tail only tests PENE==0; the completed response
      // supplies its activity predicate, not an invented geometric penetration.
      value[8][i] = response.contact_active ? 1. : 0.;
    }
    rd_assembly(static_cast<int>(incoming.size()), static_cast<int>(count),
        node[0].data(), node[1].data(), node[2].data(), node[3].data(), node[4].data(),
        value[0].data(), value[1].data(), value[2].data(), value[3].data(), value[4].data(),
        value[5].data(), value[6].data(), value[7].data(), value[8].data(), a.data(), stiffness.data());
    first = end;
  }
  auto out = incoming;
  for (std::size_t n = 0; n < out.size(); ++n)
    out[n] = {{a[3*n],a[3*n+1],a[3*n+2]},stiffness[n]};
  return out;
}
} // namespace type25_assembly_test
