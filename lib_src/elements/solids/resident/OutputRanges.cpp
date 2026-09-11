// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tl::fea::solids {
namespace {
template<class T> bool Range(const void* output, std::size_t bytes,
    const T* source, std::size_t count = 1) noexcept {
  return !count || trial_identity::Disjoint(output, bytes, source, count * sizeof(T));
}
template<class T> bool Output(T* pointer, std::size_t count, std::size_t expected) noexcept {
  if (count != expected) return false;
  if (!count) return pointer == nullptr;
  const auto address = reinterpret_cast<std::uintptr_t>(pointer);
  return pointer && address % alignof(T) == 0 && count <= (UINTPTR_MAX - address) / sizeof(T);
}
} // namespace
bool Batch::Impl::OutputDisjoint(const void* output, std::size_t bytes) const noexcept {
  const auto* domain = model.domain();
  const auto* coefficients = model.contributions();
  if (!Range(output, bytes, this) || !Range(output, bytes, static_cast<const unsigned char*>(staging.data()), staging.bytes()) ||
      !Range(output, bytes, domain) || !Range(output, bytes, domain->nodes().data(), domain->node_count()) ||
      !Range(output, bytes, coefficients) ||
      !Range(output, bytes, coefficients->parents().data(), coefficients->parents().size()) ||
      !Range(output, bytes, model.solid18().data(), model.solid18().size()) ||
      !Range(output, bytes, model.solid24().data(), model.solid24().size()) ||
      !Range(output, bytes, model.solid6z().data(), model.solid6z().size()) ||
      !Range(output, bytes, model.materials36().data(), model.materials36().size()) ||
      !Range(output, bytes, model.materials42().data(), model.materials42().size()) ||
      !Range(output, bytes, model.solid18_law44().data(), model.solid18_law44().size()) ||
      !Range(output, bytes, model.solid18_law90().data(), model.solid18_law90().size()) ||
      !Range(output, bytes, model.materials44().data(), model.materials44().size()) ||
      !Range(output, bytes, model.materials90().data(), model.materials90().size())) return false;
  for (const auto& material : model.materials36()) {
    const auto& curve = material.value.curve;
    if (!Range(output, bytes, curve.plastic_strain, curve.count) ||
        !Range(output, bytes, curve.yield_stress_pa, curve.count)) return false;
  }
  for (const auto& material : model.materials44()) {
    const auto& curve = material.value.curve;
    if (!Range(output, bytes, curve.plastic_strain, curve.count) ||
        !Range(output, bytes, curve.yield_stress_pa, curve.count)) return false;
  }
  for (const auto& material : model.materials90()) {
    const auto curve = material.value.curve();
    if (!Range(output, bytes, curve.compression_strain, curve.count) ||
        !Range(output, bytes, curve.stress_pa, curve.count)) return false;
  }
  return true;
}
bool Batch::Impl::OutputBuffers(ResultBuffers buffers, const void* input,
    std::size_t input_bytes, const void* batch, std::size_t batch_bytes) const noexcept {
  if (!Output(buffers.solid18, buffers.count18, model.solid18().size()) ||
      !Output(buffers.solid24, buffers.count24, model.solid24().size()) ||
      !Output(buffers.solid6z, buffers.count6z, model.solid6z().size()) ||
      !Output(buffers.solid18_law44, buffers.count18_law44, model.solid18_law44().size()) ||
      !Output(buffers.solid18_law90, buffers.count18_law90, model.solid18_law90().size())) return false;
  const void* pointers[]{buffers.solid18, buffers.solid24, buffers.solid6z,
      buffers.solid18_law44, buffers.solid18_law90};
  const std::size_t bytes[]{buffers.count18 * sizeof(Result18), buffers.count24 * sizeof(Result24),
      buffers.count6z * sizeof(Result6z), buffers.count18_law44 * sizeof(Result18Law44),
      buffers.count18_law90 * sizeof(Result18Law90)};
  for (unsigned f = 0; f < 5; ++f) {
    if (!bytes[f]) continue;
    if (!OutputDisjoint(pointers[f], bytes[f]) ||
        !trial_identity::Disjoint(pointers[f], bytes[f], input, input_bytes) ||
        !trial_identity::Disjoint(pointers[f], bytes[f], batch, batch_bytes)) return false;
    for (unsigned prior = 0; prior < f; ++prior) {
      if (bytes[prior] && !trial_identity::Disjoint(pointers[f], bytes[f], pointers[prior], bytes[prior]))
        return false;
    }
  }
  return true;
}
} // namespace tl::fea::solids
