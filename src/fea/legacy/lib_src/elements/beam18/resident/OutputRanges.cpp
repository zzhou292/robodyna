// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tl::fea::beam18 {
namespace {
template<class T> bool Range(const void* output, std::size_t bytes,
    const T* source, std::size_t count = 1) noexcept {
  return !count || trial_identity::Disjoint(output, bytes, source, count * sizeof(T));
}
}
bool Batch::Impl::OutputDisjoint(const void* output, std::size_t bytes) const noexcept {
  const auto* domain = model.domain();
  if (!Range(output, bytes, this) ||
      !Range(output, bytes, static_cast<const unsigned char*>(staging.data()), staging.bytes()) ||
      !Range(output, bytes, domain) || !Range(output, bytes, domain->nodes().data(), domain->node_count()) ||
      !Range(output, bytes, model.parents().data(), model.parents().size()) ||
      !Range(output, bytes, model.materials().data(), model.materials().size())) return false;
  for (const auto& m : model.materials()) {
    const auto& c = m.value.curve;
    if (!Range(output, bytes, c.plastic_strain, c.count) || !Range(output, bytes, c.yield_stress_pa, c.count))
      return false;
  }
  return true;
}
bool Batch::Impl::OutputBuffers(ResultBuffer buffer, const void* input, std::size_t input_bytes,
    const void* batch, std::size_t batch_bytes) const noexcept {
  const auto address = reinterpret_cast<std::uintptr_t>(buffer.values);
  if (buffer.count != model.parents().size() || !buffer.values || address % alignof(Result) ||
      buffer.count > (UINTPTR_MAX - address) / sizeof(Result)) return false;
  const auto bytes = buffer.count * sizeof(Result);
  return OutputDisjoint(buffer.values, bytes) &&
      trial_identity::Disjoint(buffer.values, bytes, input, input_bytes) &&
      trial_identity::Disjoint(buffer.values, bytes, batch, batch_bytes);
}
} // namespace tl::fea::beam18
