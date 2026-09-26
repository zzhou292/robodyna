// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../search_startup/Types.h"
namespace tlfea::contact::radioss_type25::tied_removal {
using Status=search_startup::Status;
enum class Finalization { Unspecified, CompactedAfterKinChk };
struct Main { std::uint32_t nodes[4]{}; }; // Domain nodes; T3 repeats slot 3 in slot 4.
struct Row {
  std::uint32_t node=UINT32_MAX;
  int local_main=0; // Actual positive, one-based finalized TYPE2 IRTLM.
  int irupt=0; // 0=CIN, otherwise PEN. Both enter PRE_I2 without filtering.
};
struct Interface {
  std::uint64_t source_id=0;
  std::uint32_t native_ordinal=0; // Actual interface storage order, not ID order.
  int level=-1; // This source profile admits resolved ILEV28 only.
  const Main* mains=nullptr;
  std::size_t main_count=0;
  const Row* rows=nullptr;
  std::size_t row_count=0;
};
// Exactly the channels reset by REMN_I2OP. Other TYPE25 history channels remain
// outside this value result and must be retained by their existing owner.
struct History {
  int irtlm[4]{};
  double penetration[5]{};
  double time[2]{};
};
struct Input {
  search_startup::Input source;
  search_startup::GeometricSnapshot geometric;
  Finalization finalization=Finalization::Unspecified;
  int tied_removal=-1; // Resolved Irem_i2=1, serial, no edge-to-edge path.
  const Interface* interfaces=nullptr;
  std::size_t interface_count=0;
  const History* history=nullptr;
  std::size_t history_count=0;
  // Native IPARI62/S_REMNODE before augmentation, possibly larger than the
  // used CSR prefix. Padding is never read or interpreted as a removal.
  std::size_t native_removal_extent=0;
};
struct Limits {
  search_startup::Limits search;
  std::size_t max_interfaces=65536, max_tied_mains=2097152, max_tied_rows=1048576;
  std::size_t max_relations=5242880, max_relation_visits=268435456;
};
struct Forecast {
  Status status=Status::InvalidInput;
  std::size_t output_bytes=0, scratch_bytes=0, removal_capacity=0, relation_capacity=0;
};
struct Snapshot {
  search_startup::Snapshot search;
  const History* history=nullptr;
  std::size_t history_count=0, added_removals=0, reset_rows=0;
  std::size_t native_removal_extent=0;
};
struct Report {
  Status status=Status::InvalidInput;
  std::size_t interface=SIZE_MAX, row=SIZE_MAX, main=SIZE_MAX;
  std::size_t required_removals=0, added_removals=0, reset_rows=0;
  bool removal_count_complete=false;
};
} // namespace tlfea::contact::radioss_type25::tied_removal
