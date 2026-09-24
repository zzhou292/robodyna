// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FacetFilters.h"
#include "Storage.h"
#include "../self_contact_filters/Environment.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <limits>

namespace tlfea::contact::self_contact_transaction {
namespace {
filters::Report Invalid(const char* message) noexcept {
  return {filters::Status::InvalidInput, message};
}
}
filters::Report FacetFilters::Initialize(const SelfContactActiveUseBinding& source,
    std::size_t pairs, std::size_t host_cap, std::size_t device_cap, cudaStream_t stream) noexcept {
  if (device_failure_.status == filters::Status::DeviceFailure) return device_failure_;
  if (initialized_) return {filters::Status::AlreadyInitialized, "Facet-filter adapter is immutable"};
  const auto count = source.facet_uses().size();
  if (!source.prepared() || !count || !source.OutputDisjoint(this, sizeof(*this)) ||
      !stream || stream == cudaStreamLegacy || stream == cudaStreamPerThread)
    return Invalid("Facet-filter adapter source or explicit stream is invalid");
  const auto forecast = Preflight(count, pairs, host_cap, device_cap);
  if (forecast.report.status != filters::Status::Ok) return forecast.report;
  // This decision precedes all filter CUDA commands, including allocation.
  // An unsupported initial environment keeps this adapter on CPU for its lifetime.
  const bool compatible = filters::CompatibleHostArithmetic();
  if (compatible) {
    if (!arena_.Initialize(forecast.scene.bytes))
      return {filters::Status::ResourceLimit, "Facet-filter compact host scene allocation failed"};
    accepted_ = arena_.Construct<filters::TriangleGeometry>(forecast.scene.accepted);
    prepared_ = arena_.Construct<filters::TriangleGeometry>(forecast.scene.prepared);
    properties_ = arena_.Construct<filters::FacetProperties>(forecast.scene.properties);
    if (!accepted_ || !prepared_ || !properties_)
      return {filters::Status::ResourceLimit, "Facet-filter compact host scene construction failed"};
    const auto initialized = batch_.Initialize({count, pairs, device_cap, host_cap}, stream);
    if (initialized.status != filters::Status::Ok) { ObserveFailure(initialized); return initialized; }
  }
  source_ = &source; facets_ = count; pairs_ = pairs; stream_ = stream;
  layout_ = forecast.scene;
  reserved_device_bytes_ = forecast.device_bytes;
  reserved_device_allocations_ = forecast.device_allocations;
  initialized_ = true; device_available_ = compatible;
  return {};
}

bool FacetFilters::OutputDisjoint(const void* output, std::size_t bytes) const noexcept {
  using tl::fea::trial_identity::Disjoint;
  if (!bytes) return true;
  return output && Disjoint(output, bytes, this, sizeof(*this)) &&
      (!arena_.bytes() || Disjoint(output, bytes, arena_.data(), arena_.bytes())) &&
      batch_.OutputDisjoint(output, bytes);
}
void FacetFilters::Discard() noexcept {
  batch_.DiscardScene();
  base_input_ = next_input_ = nullptr;
  motion_ = nullptr; bounds_ = nullptr; chunk_ = nullptr;
  chunk_count_ = 0; span_begin_ = SIZE_MAX; span_end_ = 0;
  scene_generation_ = 0; device_scene_ = false; phase_ = Phase::None;
}
filters::Report FacetFilters::PrepareScene(const CurrentFixedTriangle* base,
    const CurrentFixedTriangle* next, const MotionSupport* motion,
    const SelfContactSweptParentBounds* bounds, Phase phase) noexcept {
  if (device_failure_.status == filters::Status::DeviceFailure) return device_failure_;
  if (!initialized_) return {filters::Status::NotInitialized, "Facet-filter adapter is not initialized"};
  // Reject any own-storage alias before revocation or compact-scene writes.
  if (!base || !next || !motion || (phase == Phase::Candidate && !bounds) ||
      !OutputDisjoint(base, facets_*sizeof(*base)) ||
      !OutputDisjoint(next, facets_*sizeof(*next)) ||
      !OutputDisjoint(motion, facets_*sizeof(*motion)) ||
      (bounds && !OutputDisjoint(bounds, facets_*sizeof(*bounds)))) {
    Discard();
    return Invalid("Facet-filter scene aliases its retained storage or is incomplete");
  }
  Discard();
  base_input_ = base; next_input_ = next; motion_ = motion; bounds_ = bounds; phase_ = phase;
  if (!device_available_ || !filters::CompatibleHostArithmetic()) return {};
  const auto uses = source_->facet_uses(); const auto parents = source_->parents();
  for (std::size_t facet = 0; facet < facets_; ++facet) {
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
      accepted_[facet].vertices[vertex] = base[facet].vertices[vertex];
      prepared_[facet].vertices[vertex] = next[facet].vertices[vertex];
    }
    const auto parent = phase == Phase::Candidate ? motion[facet].parent : uses[facet].parent;
    // Preserve per-pair parent error order. An invalid unreferenced parent is
    // not promoted to an eager scene error or silently admitted by the device.
    properties_[facet].half_thickness = parent < parents.size()
        ? parents[parent].reference_half_thickness_m : std::numeric_limits<double>::quiet_NaN();
    properties_[facet].complete_rigid_group = motion[facet].complete_rigid_group;
  }
  const auto uploaded = batch_.Upload({accepted_, prepared_, properties_, facets_}, stream_);
  if (uploaded.status != filters::Status::Ok) { ObserveFailure(uploaded); Discard(); return uploaded; }
  // A zero-pair query publishes the current generation without issuing CUDA work.
  const auto published = batch_.Accepted({}, stream_);
  if (published.status == filters::Status::UnsupportedEnvironment) return {};
  if (published.status != filters::Status::Ok) { ObserveFailure(published); Discard(); return published; }
  const auto view = batch_.results();
  if (!view.complete || view.count) { Discard(); return Invalid("Facet-filter scene publication is incomplete"); }
  scene_generation_ = view.scene_generation; device_scene_ = true;
  return {};
}
filters::Report FacetFilters::AcceptedScene(const CurrentFixedTriangle* accepted,
    const MotionSupport* motion) noexcept {
  return PrepareScene(accepted, accepted, motion, nullptr, Phase::Accepted);
}
filters::Report FacetFilters::CandidateScene(const CurrentFixedTriangle* accepted,
    const CurrentFixedTriangle* prepared, const MotionSupport* motion,
    const SelfContactSweptParentBounds* bounds) noexcept {
  return PrepareScene(accepted, prepared, motion, bounds, Phase::Candidate);
}
void FacetFilters::BeginCandidateChunk(const FixedTrianglePair* pairs, std::size_t count) noexcept {
  chunk_ = pairs; chunk_count_ = count; span_begin_ = SIZE_MAX; span_end_ = 0;
}
FacetPrismReply FacetFilters::PrismAt(std::size_t ordinal) noexcept {
  FacetPrismReply result;
  result.supplied = true;
  if (device_failure_.status == filters::Status::DeviceFailure) {
    result.report = device_failure_; return result;
  }
  if (!initialized_) {
    result.report = {filters::Status::NotInitialized, "Facet-filter adapter is not initialized"};
    return result;
  }
  if (phase_ != Phase::Candidate) {
    result.report = {filters::Status::NoScene, "Facet-filter candidate scene is not current"};
    return result;
  }
  if (!chunk_ || chunk_count_ > pairs_ || ordinal >= chunk_count_ ||
      !OutputDisjoint(chunk_, chunk_count_*sizeof(*chunk_))) {
    result.report = Invalid("Facet-filter candidate chunk is not a current disjoint borrow");
    Discard(); return result;
  }
  if (!device_scene_ || !filters::CompatibleHostArithmetic()) {
    span_begin_ = SIZE_MAX; span_end_ = 0;
    result.supplied = false;
    return result;
  }
  if (span_begin_ == SIZE_MAX || ordinal < span_begin_ || ordinal >= span_end_) {
    const auto end = LinearFacetSpanEnd(chunk_, chunk_count_, ordinal, motion_, bounds_,
                                        facets_, source_->parents().size());
    if (end == ordinal) {
      result.report = Invalid("Facet-filter linear span does not begin with an admitted affine pair");
      Discard(); return result;
    }
    result.report = batch_.Linear({chunk_+ordinal, end-ordinal}, FacetPrismAxisLimit::VertexVertex, stream_);
    if (result.report.status == filters::Status::UnsupportedEnvironment) {
      result.report = {}; result.supplied = false; span_begin_ = SIZE_MAX; span_end_ = 0;
      return result;
    }
    if (result.report.status != filters::Status::Ok) { ObserveFailure(result.report); Discard(); return result; }
    span_begin_ = ordinal; span_end_ = end;
  }
  const auto view = batch_.results();
  if (!view.complete || view.count != span_end_-span_begin_ ||
      view.scene_generation != scene_generation_) {
    result.report = Invalid("Facet-filter linear publication differs from the current scene/span");
    Discard(); return result;
  }
  result.value = view.data[ordinal-span_begin_];
  return result;
}
}  // namespace tlfea::contact::self_contact_transaction
