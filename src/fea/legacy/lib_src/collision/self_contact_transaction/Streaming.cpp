// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include <algorithm>

namespace tlfea::contact::self_contact_transaction {
namespace {

using S = SelfContactTransactionStatus;

SelfContactTransactionReport Failure(
    S status, const char* message,
    std::size_t pair = SIZE_MAX) noexcept {
  SelfContactTransactionReport result;
  result.status = status;
  result.pair = pair;
  result.message = message;
  return result;
}

FixedTriangleKey Key(const FixedContactFacet& value) noexcept {
  return {value.source_instance_id, value.source.source_parent_id,
          value.level, value.local_facet};
}

int PairCompare(const FixedContactFacet* descriptors,
                FixedTrianglePair a, FixedTrianglePair b) noexcept {
  int order = fixed_triangle_features::Compare(
      Key(descriptors[a.first]), Key(descriptors[b.first]));
  if (!order)
    order = fixed_triangle_features::Compare(
        Key(descriptors[a.second]), Key(descriptors[b.second]));
  return order;
}

FixedTrianglePair Current(const FacetPairCursor& cursor) noexcept {
  return {cursor.first, cursor.second};
}

}  // namespace

SelfContactTransactionReport StreamingCandidateSource::Initialize(
    const FixedContactFacet* descriptors, std::size_t facets,
    FacetPairCursor* cursors, std::size_t cursor_capacity,
    std::uint32_t* heap, std::size_t heap_capacity,
    FixedTrianglePair* chunk, std::size_t chunk_capacity,
    std::size_t complete_pair_capacity) noexcept {
  if (initialized_)
    return Failure(S::AlreadyInitialized,
                   "Streaming candidate source is immutable");
  if (!descriptors || !facets || !cursors || !cursor_capacity ||
      !heap || heap_capacity < cursor_capacity ||
      !chunk || !chunk_capacity || !complete_pair_capacity ||
      cursor_capacity > UINT32_MAX || facets > UINT32_MAX)
    return Failure(S::InvalidInput,
                   "Streaming candidate source storage is incomplete");
  descriptors_ = descriptors;
  facets_ = facets;
  cursors_ = cursors;
  cursor_capacity_ = cursor_capacity;
  heap_ = heap;
  heap_capacity_ = heap_capacity;
  chunk_ = chunk;
  chunk_capacity_ = chunk_capacity;
  complete_pair_capacity_ = complete_pair_capacity;
  initialized_ = true;
  return {};
}

SelfContactTransactionReport StreamingCandidateSource::Begin(
    const SelfContactPairKey* keys, std::size_t key_count,
    const std::uint32_t* surface_to_active,
    std::size_t surface_parents,
    const std::uint32_t* parent_facet_offsets,
    std::size_t parents, SelfContactActivityView activity) noexcept {
  if (!initialized_)
    return Failure(S::NotInitialized,
                   "Streaming candidate source is not initialized");
  ++identity_;
  if (!identity_) ++identity_;
  active_ = false;
  heap_count_ = parent_pair_count_ = required_facet_pairs_ = 0;
  emitted_facet_pairs_ = chunks_ = 0;
  have_previous_ = false;
  if ((key_count && !keys) || key_count > cursor_capacity_ ||
      !surface_to_active || !surface_parents ||
      !parent_facet_offsets || !parents ||
      !activity.base || !activity.current ||
      activity.parent_count != parents ||
      parent_facet_offsets[0] != 0 ||
      parent_facet_offsets[parents] != facets_)
    return Failure(S::InvalidInput,
                   "Streaming parent/facet source is incomplete");
  for (std::size_t parent = 0; parent < parents; ++parent) {
    if (activity.base[parent] > 1 ||
        activity.current[parent] > activity.base[parent])
      return Failure(S::ActivityFailure,
                     "Streaming facet activity is invalid", parent);
    const auto begin = parent_facet_offsets[parent];
    const auto end = parent_facet_offsets[parent + 1];
    if (begin >= end || end > facets_)
      return Failure(S::IdentityMismatch,
                     "Streaming parent facet range is invalid", parent);
    for (std::uint32_t facet = begin + 1; facet < end; ++facet)
      if (fixed_triangle_features::Compare(
              Key(descriptors_[facet - 1]),
              Key(descriptors_[facet])) >= 0)
        return Failure(S::IdentityMismatch,
                       "Parent facets are not immutable canonical order",
                       facet);
  }

  std::size_t cursor_count = 0;
  std::size_t required = 0;
  for (std::size_t pair = 0; pair < key_count; ++pair) {
    if ((pair && keys[pair - 1] >= keys[pair]) ||
        FirstSurfaceParent(keys[pair]) >= surface_parents ||
        SecondSurfaceParent(keys[pair]) >= surface_parents ||
        FirstSurfaceParent(keys[pair]) >=
            SecondSurfaceParent(keys[pair]))
      return Failure(S::IdentityMismatch,
                     "Broadphase keys are not a complete canonical roster",
                     pair);
    const auto first =
        surface_to_active[FirstSurfaceParent(keys[pair])];
    const auto second =
        surface_to_active[SecondSurfaceParent(keys[pair])];
    if (first >= parents || second >= parents || first == second)
      return Failure(S::IdentityMismatch,
                     "Broadphase pair has no selected parent mapping",
                     pair);
    if (!activity.current[first] || !activity.current[second])
      continue;
    std::uint32_t first_begin = parent_facet_offsets[first];
    std::uint32_t first_end = parent_facet_offsets[first + 1];
    std::uint32_t second_begin = parent_facet_offsets[second];
    std::uint32_t second_end = parent_facet_offsets[second + 1];
    if (fixed_triangle_features::Compare(
            Key(descriptors_[second_begin]),
            Key(descriptors_[first_begin])) < 0) {
      std::swap(first_begin, second_begin);
      std::swap(first_end, second_end);
    }
    const std::size_t first_count = first_end - first_begin;
    const std::size_t second_count = second_end - second_begin;
    if (first_count &&
        second_count > (complete_pair_capacity_ - required) / first_count)
      return Failure(S::ResourceLimit,
                     "Complete streamed facet-pair census exceeds cap",
                     pair);
    required += first_count * second_count;
    auto& cursor = cursors_[cursor_count];
    cursor = {first_begin, first_end, second_begin, second_end,
              first_begin, second_begin,
              static_cast<std::uint32_t>(pair)};
    heap_[cursor_count] = static_cast<std::uint32_t>(cursor_count);
    ++cursor_count;
  }

  const auto less = [&](std::uint32_t a, std::uint32_t b) noexcept {
    const int order = PairCompare(
        descriptors_, Current(cursors_[a]), Current(cursors_[b]));
    return order < 0 ||
        (!order && cursors_[a].parent_pair < cursors_[b].parent_pair);
  };
  heap_count_ = cursor_count;
  if (heap_count_ > heap_capacity_)
    return Failure(S::ResourceLimit,
                   "Streaming heap capacity is incomplete");
  const auto sift_down = [&](std::size_t root) noexcept {
    for (;;) {
      const auto left = 2 * root + 1;
      if (left >= heap_count_) return;
      auto child = left;
      const auto right = left + 1;
      if (right < heap_count_ && less(heap_[right], heap_[left]))
        child = right;
      if (!less(heap_[child], heap_[root])) return;
      std::swap(heap_[root], heap_[child]);
      root = child;
    }
  };
  if (heap_count_)
    for (std::size_t i = heap_count_ / 2; i != 0; --i)
      sift_down(i - 1);

  parent_pair_count_ = key_count;
  required_facet_pairs_ = required;
  active_ = true;
  return {};
}

SelfContactTransactionReport StreamingCandidateSource::Next(
    const FixedTrianglePair** output, std::size_t* count) noexcept {
  if (!active_ || !output || !count)
    return Failure(S::InvalidInput,
                   "Streaming chunk query is not active");
  *output = nullptr;
  *count = 0;
  std::size_t written = 0;
  const auto less = [&](std::uint32_t a, std::uint32_t b) noexcept {
    const int order = PairCompare(
        descriptors_, Current(cursors_[a]), Current(cursors_[b]));
    return order < 0 ||
        (!order && cursors_[a].parent_pair < cursors_[b].parent_pair);
  };
  const auto sift_down = [&](std::size_t root) noexcept {
    for (;;) {
      const auto left = 2 * root + 1;
      if (left >= heap_count_) return;
      auto child = left;
      const auto right = left + 1;
      if (right < heap_count_ && less(heap_[right], heap_[left]))
        child = right;
      if (!less(heap_[child], heap_[root])) return;
      std::swap(heap_[root], heap_[child]);
      root = child;
    }
  };

  while (written < chunk_capacity_ && heap_count_) {
    auto& cursor = cursors_[heap_[0]];
    const auto pair = Current(cursor);
    if (have_previous_ &&
        PairCompare(descriptors_, previous_, pair) >= 0) {
      active_ = false;
      return Failure(S::IdentityMismatch,
                     "Canonical facet-pair stream repeats or descends",
                     emitted_facet_pairs_);
    }
    chunk_[written++] = pair;
    previous_ = pair;
    have_previous_ = true;
    ++emitted_facet_pairs_;

    ++cursor.second;
    if (cursor.second == cursor.second_end) {
      cursor.second = cursor.second_begin;
      ++cursor.first;
    }
    if (cursor.first == cursor.first_end) {
      heap_[0] = heap_[--heap_count_];
    }
    if (heap_count_) sift_down(0);
  }
  if (written) ++chunks_;
  *output = written ? chunk_ : nullptr;
  *count = written;
  return {};
}

SelfContactTransactionReport StreamingCandidateSource::Finish(
    StreamingCandidateSourceReceipt* output) noexcept {
  if (!active_ || !output || heap_count_ ||
      emitted_facet_pairs_ != required_facet_pairs_)
    return Failure(S::IdentityMismatch,
                   "Streaming source cannot certify a partial traversal",
                   emitted_facet_pairs_);
  StreamingCandidateSourceReceipt receipt;
  receipt.source_ = this;
  receipt.identity_ = identity_;
  receipt.parent_pairs_ = parent_pair_count_;
  receipt.required_facet_pairs_ = required_facet_pairs_;
  receipt.emitted_facet_pairs_ = emitted_facet_pairs_;
  receipt.chunks_ = chunks_;
  *output = receipt;
  active_ = false;
  return {};
}

bool StreamingCandidateSource::Authenticates(
    const StreamingCandidateSourceReceipt& receipt) const noexcept {
  return receipt.complete() && receipt.source_ == this &&
      receipt.identity_ == identity_ &&
      receipt.parent_pairs_ == parent_pair_count_ &&
      receipt.required_facet_pairs_ == required_facet_pairs_ &&
      receipt.emitted_facet_pairs_ == emitted_facet_pairs_ &&
      receipt.chunks_ == chunks_ && !active_ && !heap_count_;
}

}  // namespace tlfea::contact::self_contact_transaction
