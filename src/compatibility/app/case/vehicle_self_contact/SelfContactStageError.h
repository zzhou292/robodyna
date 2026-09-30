#pragma once

#include "lib_src/collision/SelfContactTransactionTypes.h"

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace crash::cases::vehicle_self_contact {

enum class SelfContactRuntimeStage : std::uint8_t {
    AcceptedAssembly,
    CandidateSeal,
};

// Preserves the complete typed TL report across the app dynamics rollback.
// required_events() is populated only for a typed exact post-discovery force-cap
// check. A compact census that exhausted its storage supplies only a lower
// bound, through required_events_lower_bound(); candidate ordinals are neither.
// Callers never infer a count from report text.
class SelfContactStageError : public std::runtime_error {
  public:
    SelfContactStageError(
        tlfea::contact::SelfContactTransactionReport report,
        SelfContactRuntimeStage stage,
        std::size_t configured_event_capacity)
        : std::runtime_error(
              std::string(StageName(stage)) + ": " + report.message),
          report_(report),
          stage_(stage),
          required_events_(
              stage == SelfContactRuntimeStage::AcceptedAssembly &&
                      report.status ==
                          tlfea::contact::SelfContactTransactionStatus::
                              ResourceLimit &&
                      report.count_kind ==
                          tlfea::contact::SelfContactTransactionCountKind::
                              ExactAcceptedEvents &&
                      report.candidate != SIZE_MAX &&
                      report.candidate > configured_event_capacity
                  ? report.candidate
                  : 0) {}

    const tlfea::contact::SelfContactTransactionReport&
    report() const noexcept {
        return report_;
    }
    SelfContactRuntimeStage stage() const noexcept {
        return stage_;
    }
    std::size_t required_events() const noexcept {
        return required_events_;
    }
    // Zero means no typed lower bound is available. A valid lower bound need
    // not exceed the force cap: the identity census can exhaust before the
    // final force requirement is known. It is never sufficient to size a retry.
    std::size_t required_events_lower_bound() const noexcept {
        return stage_ == SelfContactRuntimeStage::AcceptedAssembly &&
                report_.status ==
                    tlfea::contact::SelfContactTransactionStatus::ResourceLimit &&
                report_.count_kind ==
                    tlfea::contact::SelfContactTransactionCountKind::
                        AcceptedEventsLowerBound &&
                report_.candidate != SIZE_MAX
            ? report_.candidate
            : 0;
    }
    static const char* StageName(
        SelfContactRuntimeStage stage) noexcept {
        return stage == SelfContactRuntimeStage::AcceptedAssembly
            ? "accepted self-contact assembly"
            : "candidate self-contact seal";
    }

  private:
    tlfea::contact::SelfContactTransactionReport report_;
    SelfContactRuntimeStage stage_;
    std::size_t required_events_ = 0;
};

}  // namespace crash::cases::vehicle_self_contact
