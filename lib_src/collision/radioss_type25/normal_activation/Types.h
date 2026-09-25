// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../selection/lifecycle/Row.h"
namespace tlfea::contact::radioss_type25::normal_activation {
namespace lifecycle=selection::lifecycle;
using Status=selection::Status;
enum class FreeRosterPolicy { Unspecified, FreshComplete };
struct Profile {
  int edge_mode=-1,foreign_rows=-1,partitions=0,neighbor_removal=-1,local_processor=0;
  FreeRosterPolicy free_roster=FreeRosterPolicy::Unspecified;
};
struct Input {
  Profile profile;
  lifecycle::SourceView source;
  const lifecycle::OptimizedRow* rows=nullptr;std::size_t row_count=0;
  // Exact OPTCD suffix, excluding the retained prefix. Count/domain checks do
  // not authenticate its completeness: the owning stage binds the actual list.
  const std::uint32_t* optimized_main_ids=nullptr;std::size_t optimized_count=0;
  // Exact ascending fresh I25FREE_BOUND list. A stale pre-deletion roster is
  // outside this profile; dynamic removal needs its native refresh lifecycle.
  const std::uint32_t* free_main_ids=nullptr;std::size_t free_count=0;
};
struct Output {
  std::uint32_t* main_active=nullptr;std::size_t main_count=0;
  std::uint32_t* node_tag=nullptr;std::size_t node_count=0;
};
struct Limits {
  std::size_t nodes=524288,mains=2097152,secondaries=524288;
  std::size_t references=4194304,incidences=8388608,optimized=8388608,removals=8388608;
};
} // namespace tlfea::contact::radioss_type25::normal_activation
