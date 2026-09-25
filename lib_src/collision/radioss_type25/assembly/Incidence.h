// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Schedule.h"
#include "lib_utils/OrderedNodeIncidence.h"
namespace tlfea::contact::radioss_type25::assembly {
namespace detail {
struct Range { std::uintptr_t begin = 0, end = 0; };
template<class T> inline bool RangeOf(const T* data, std::size_t count, Range& range) {
  if (!count) { range = {}; return data == nullptr; }
  const auto begin = reinterpret_cast<std::uintptr_t>(data);
  if (!data || begin % alignof(T) || count > SIZE_MAX / sizeof(T) ||
      count * sizeof(T) > UINTPTR_MAX - begin) return false;
  range = {begin, begin + count * sizeof(T)};
  return true;
}
inline bool Overlap(Range a, Range b) { return a.begin < b.end && b.begin < a.end; }
} // namespace detail
// Caller-owned host arrays; no allocation, device launch, or ownership transfer.
// Every slot (including duplicate/zero-weight slots) is retained. Input/output
// overlap, invalid maps/schedules/capacities fail before either output is changed.
inline bool BuildIncidence(const Connectivity* rows, Schedule schedule,
    std::size_t nodes, std::uint32_t* offsets, std::size_t offset_count,
    std::uint32_t* occurrences, std::size_t occurrence_count) noexcept {
  if (!nodes || nodes >= UINT32_MAX || schedule.row_count > UINT32_MAX / 5 ||
      schedule.cohort_count > schedule.row_count || offset_count != nodes + 1 ||
      occurrence_count != 5 * schedule.row_count) return false;
  detail::Range inputs[2], outputs[2];
  if (!detail::RangeOf(rows, schedule.row_count, inputs[0]) ||
      !detail::RangeOf(schedule.cohort_ends, schedule.cohort_count, inputs[1]) ||
      !detail::RangeOf(offsets, offset_count, outputs[0]) ||
      !detail::RangeOf(occurrences, occurrence_count, outputs[1])) return false;
  if (detail::Overlap(outputs[0], outputs[1])) return false;
  for (auto output : outputs)
    for (auto input : inputs) if (detail::Overlap(output, input)) return false;
  if (!schedule.row_count) {
    if (schedule.cohort_count) return false;
    for (std::size_t node = 0; node <= nodes; ++node) offsets[node] = 0;
    return true;
  }
  if (!schedule.cohort_count || schedule.cohort_ends[schedule.cohort_count - 1] !=
      schedule.row_count) return false;
  std::uint32_t prior = 0;
  for (std::size_t cohort = 0; cohort < schedule.cohort_count; ++cohort) {
    const auto end = schedule.cohort_ends[cohort];
    if (end <= prior || end > schedule.row_count) return false;
    prior = end;
  }
  return tl::util::BuildOrderedNodeIncidence<1>(occurrence_count, nodes,
      [=](std::size_t rank, unsigned) {
        Occurrence value;
        return DecodeOccurrence(schedule, static_cast<std::uint32_t>(rank), &value) ?
            EndpointNode(rows[value.row], value.slot) : UINT32_MAX;
      }, offsets, offset_count, occurrences, occurrence_count);
}
} // namespace tlfea::contact::radioss_type25::assembly
