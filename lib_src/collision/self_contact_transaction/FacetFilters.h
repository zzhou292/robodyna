// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../SelfContactTransactionTypes.h"
#include "../self_contact_filters/Batch.h"
#include "lib_utils/BoundedArena.h"

namespace tlfea::contact::self_contact_transaction {
struct MotionSupport;
namespace filters = self_contact_filters;
struct FacetFilterLayout {
  tl::util::ArenaRegion accepted, prepared, properties;
  std::size_t bytes = 0;
};
struct FacetFilterForecast {
  filters::Report report;
  filters::Forecast batch;
  FacetFilterLayout scene;
  std::size_t owned_host_bytes = 0, startup_host_bytes = 0;
  std::size_t device_bytes = 0, device_allocations = 0;
};
struct FacetPrismReply {
  filters::Report report;
  filters::PairResult value;
  bool supplied = false;
};
// One optional numerical adapter, never a physical owner or receipt issuer.
// Borrowed scene/chunk pointers are lexical to the authenticated transaction
// attempt. The immutable active-use binding outlives this owned adapter.
class FacetFilters {
 public:
  static FacetFilterForecast Preflight(std::size_t facets, std::size_t pairs,
      std::size_t host_cap, std::size_t device_cap) noexcept;
  filters::Report Initialize(const SelfContactActiveUseBinding&, std::size_t pairs,
      std::size_t host_cap, std::size_t device_cap, cudaStream_t) noexcept;
  filters::Report AcceptedScene(const CurrentFixedTriangle*, const MotionSupport*) noexcept;
  filters::Report CandidateScene(const CurrentFixedTriangle*, const CurrentFixedTriangle*,
      const MotionSupport*, const SelfContactSweptParentBounds*) noexcept;
  SelfContactTransactionReport AcceptedPairs(FixedTrianglePair*, std::size_t*) noexcept;
  void BeginCandidateChunk(const FixedTrianglePair*, std::size_t) noexcept;
  FacetPrismReply PrismAt(std::size_t ordinal) noexcept;
  void Discard() noexcept;
  SelfContactFacetFilterInitialization initialization_mode() const noexcept {
    if (!initialized_) return SelfContactFacetFilterInitialization::NotInitialized;
    return device_available_ ? SelfContactFacetFilterInitialization::Cuda
                            : SelfContactFacetFilterInitialization::UnsupportedHostArithmetic;
  }
  // Forecast includes optional capacity even when unsupported startup math
  // selected lifetime CPU fallback; allocation telemetry reports actual storage.
  tl::fea::NodalAllocationInfo UnallocatedDevice() const noexcept {
    return device_available_ ? tl::fea::NodalAllocationInfo{} :
        tl::fea::NodalAllocationInfo{reserved_device_bytes_, reserved_device_allocations_};
  }
  bool OutputDisjoint(const void*, std::size_t) const noexcept;
 private:
  enum class Phase { None, Accepted, Candidate };
  filters::Report PrepareScene(const CurrentFixedTriangle*, const CurrentFixedTriangle*,
      const MotionSupport*, const SelfContactSweptParentBounds*, Phase) noexcept;
  void ObserveFailure(const filters::Report& report) noexcept {
    if (report.status == filters::Status::DeviceFailure &&
        device_failure_.status != filters::Status::DeviceFailure)
      device_failure_ = report;
  }
  filters::Report device_failure_;
  filters::Batch batch_;
  tl::util::HostArena arena_;
  FacetFilterLayout layout_;
  const SelfContactActiveUseBinding* source_ = nullptr;
  filters::TriangleGeometry* accepted_ = nullptr;
  filters::TriangleGeometry* prepared_ = nullptr;
  filters::FacetProperties* properties_ = nullptr;
  const CurrentFixedTriangle* base_input_ = nullptr;
  const CurrentFixedTriangle* next_input_ = nullptr;
  const MotionSupport* motion_ = nullptr;
  const SelfContactSweptParentBounds* bounds_ = nullptr;
  const FixedTrianglePair* chunk_ = nullptr;
  std::size_t facets_ = 0, pairs_ = 0, chunk_count_ = 0;
  std::size_t reserved_device_bytes_ = 0, reserved_device_allocations_ = 0;
  std::size_t span_begin_ = SIZE_MAX, span_end_ = 0;
  std::uint64_t scene_generation_ = 0;
  cudaStream_t stream_ = nullptr;
  bool initialized_ = false, device_available_ = false, device_scene_ = false;
  Phase phase_ = Phase::None;
};
// Pure lookahead stops before any nonlinear/excluded/error row. It does not
// consume work, validate a policy or reorder the caller's serial decisions.
std::size_t LinearFacetSpanEnd(const FixedTrianglePair*, std::size_t count,
    std::size_t first, const MotionSupport*, const SelfContactSweptParentBounds*,
    std::size_t facets, std::size_t parents) noexcept;
SelfContactTransactionReport FacetFilterFailure(const filters::Report&,
    std::size_t pair = SIZE_MAX) noexcept;

// The scalar lambda is the original call at its original serial position.
// Device execution is confined to a current contiguous linear span; a failure
// never silently selects the scalar path.
template <class Scalar>
bool OptionalFacetPrism(FacetFilters* filters, std::size_t ordinal,
    Scalar scalar, SelfContactFacetPrismSeparationAxis* axis, bool* valid,
    self_contact_filters::Report* report) noexcept {
  if (!filters) return scalar();
  const auto reply = filters->PrismAt(ordinal);
  *report = reply.report;
  if (reply.report.status != self_contact_filters::Status::Ok) return false;
  if (!reply.supplied) return scalar();
  *axis = reply.value.axis;
  *valid = reply.value.status == SelfContactFacetFilterStatus::Ok;
  return reply.value.separated;
}
}  // namespace tlfea::contact::self_contact_transaction
