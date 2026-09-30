// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/initial_source/Types.h"
#include <memory>
struct CUstream_st;
namespace tlfea::contact::radioss_type25 {
namespace runtime_detail { struct InitialSeedAccess; }
namespace qualification { struct InitialSourceAccess; }
namespace initial_source {
class DeviceSeed;
// Immutable owning host preparation. This is not an accepted owner state or a
// history seed. It keeps the genuinely derived final removal CSR and copied
// operands alive through the later owner-stream initialization.
class PreparedSource {
 public:
  PreparedSource() noexcept;
  ~PreparedSource();
  PreparedSource(PreparedSource&&) noexcept;
  PreparedSource& operator=(PreparedSource&&) noexcept;
  PreparedSource(const PreparedSource&)=delete;
  PreparedSource& operator=(const PreparedSource&)=delete;
  bool prepared() const noexcept;
  SeedIdentity identity() const noexcept;
  Forecast forecast() const noexcept;
  FinalRemovalView removals() const noexcept;
 private:
  struct Impl;std::unique_ptr<Impl> impl_;
  friend Report PrepareSource(const Input&,Limits,PreparedSource&) noexcept;
  friend Report Prepare(const PreparedSource&,CUstream_st*,DeviceSeed&) noexcept;
  friend struct runtime_detail::InitialSeedAccess;
  friend struct qualification::InitialSourceAccess;
};
// Startup-only immutable device value. No public row setter, host history
// mirror, callback or physical publication. Source/clock authority is checked
// separately by the existing Transaction's initializer.
class DeviceSeed {
 public:
  DeviceSeed() noexcept;
  ~DeviceSeed();
  DeviceSeed(DeviceSeed&&) noexcept;
  DeviceSeed& operator=(DeviceSeed&&) noexcept;
  DeviceSeed(const DeviceSeed&)=delete;
  DeviceSeed& operator=(const DeviceSeed&)=delete;
  bool prepared() const noexcept;
  SeedIdentity identity() const noexcept;
  Diagnostics diagnostics() const noexcept;
 private:
  struct Impl;std::unique_ptr<Impl> impl_;
  friend Report Prepare(const Input&,Limits,CUstream_st*,DeviceSeed&) noexcept;
  friend Report Prepare(const PreparedSource&,CUstream_st*,DeviceSeed&) noexcept;
  friend struct runtime_detail::InitialSeedAccess;
  friend struct qualification::InitialSourceAccess;
};
Forecast Preflight(const Input&,Limits={}) noexcept;
// Forecast precedes any source-sized allocation. The writable output must be
// a valid distinct object, disjoint from every borrowed input range.
Report PrepareSource(const Input&,Limits,PreparedSource&) noexcept;
Report Prepare(const PreparedSource&,CUstream_st*,DeviceSeed&) noexcept;
// Borrowed host inputs stay alive/immutable through the synchronous return.
// All owned GPU work is on the supplied stream and drained before publication
// or cleanup. Caller serializes source preparation/owner lifetime. A failure
// preserves an already prepared output. No source-sized host per-pair loop.
Report Prepare(const Input&,Limits,CUstream_st*,DeviceSeed&) noexcept;
}
}
