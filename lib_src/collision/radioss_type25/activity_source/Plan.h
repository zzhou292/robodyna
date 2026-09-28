// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "../startup/PostGapmTypes.h"
#include <memory>
namespace tlfea::contact::radioss_type25::activity_source {
namespace detail { struct Storage; }
// Immutable, bounded startup ownership. No masks, history, force, physical
// clock, node deletion or runtime activity authority is constructed here.
// All non-QEPH/T3 activity must remain positive under the runtime's existing
// participant guards. Element mass and bare rigid/CIN membership add no support.
class Plan {
 public:
  Plan(); ~Plan();
  Plan(const Plan&)=delete; Plan& operator=(const Plan&)=delete;
  static Forecast Preflight(const tl::fea::ShellPhysicalBinding&,
      const ContactSourceInput&,Controls,Limits={}) noexcept;
  static Forecast Preflight(const tl::fea::ShellPhysicalBinding&,
      const startup::MixedSidesSnapshot&,const startup::PostGapmTopology&,
      Controls,Limits={}) noexcept;
  static Forecast Preflight(const tl::fea::ShellPhysicalBinding&,
      const startup::Snapshot&,Controls,Limits={}) noexcept;
  TransactionReport Initialize(const tl::fea::ShellPhysicalBinding&,
      const ContactSourceInput&,Controls,Limits={}) noexcept;
  TransactionReport Initialize(const tl::fea::ShellPhysicalBinding&,
      const startup::MixedSidesSnapshot&,const startup::PostGapmTopology&,
      Controls,Limits={}) noexcept;
  TransactionReport Initialize(const tl::fea::ShellPhysicalBinding&,
      const startup::Snapshot&,Controls,Limits={}) noexcept;
  bool initialized() const noexcept;
  bool Matches(const tl::fea::ShellPhysicalBinding&) const noexcept;
  View view() const noexcept;
  Forecast forecast() const noexcept;
 private:
  std::unique_ptr<detail::Storage> storage_;
};
} // namespace tlfea::contact::radioss_type25::activity_source
