#pragma once
#include "TiedSearchFinalized.h"
#include "modelio/tied_shell/TiedClassificationContext.h"
#include "lib_src/constraints/tied_shell/TiedClassification.h"

namespace crash::cases::vehicle_startup {
struct TiedClassificationLimits {
    std::size_t host_bytes = 512 * 1024 * 1024;
    native_search::ClassificationLimits native = [] {
        native_search::ClassificationLimits value;
        value.max_host_bytes = 64 * 1024 * 1024;
        return value;
    }();
};
struct TiedClassificationForecast {
    std::size_t source_context_reservation_bytes = 0;
    std::size_t distinct_finalized_payload_bytes = 0;
    std::size_t input_staging_bytes = 0, result_payload_bytes = 0;
    std::size_t native_reservation_bytes = 0, total_host_bytes = 0;
};
struct ClassifiedTiedSlave {
    tied::SourceId source_node_id = 0;
    std::uint32_t original_nsv_row = 0;
    std::int32_t irupt = 0;
    native_search::NativeKinematics kinematics;
};
struct TiedClassificationData {
    std::vector<ClassifiedTiedSlave> slaves; // Native compact NSV order.
    std::array<std::int32_t, 8192> interface_decode{};
    std::size_t cin_count = 0, penalty_count = 0;
    std::uint64_t kinset_warning_count = 0, penalty_warning_count = 0;
    std::size_t owned_payload_bytes = 0;
    native_search::ClassificationPhase phase = native_search::ClassificationPhase::Empty;
};
// Original-source observed slave result, at ITAGSL2 before KINCHK. Unobserved
// projected nodes are not exposed as a complete native starter condition map.
class TiedSearchClassification {
  public:
    static TiedClassificationForecast Forecast(const TiedSearchFinalized&,
        const tied::TiedClassificationContext&, TiedClassificationLimits = {});
    static TiedSearchClassification Prepare(const TiedSearchFinalized&,
        const tied::TiedClassificationContext&, TiedClassificationLimits = {});
    TiedSearchClassification(const TiedSearchClassification&) noexcept = default;
    TiedSearchClassification(TiedSearchClassification&& other) noexcept : storage_(other.storage_) {}
    TiedSearchClassification& operator=(const TiedSearchClassification&) = delete;
    TiedSearchClassification& operator=(TiedSearchClassification&&) = delete;
    const TiedSearchFinalized& finalized() const noexcept;
    const tied::TiedClassificationContext& context() const noexcept;
    const TiedClassificationData& data() const noexcept;
    const TiedClassificationForecast& forecast() const noexcept;
    TiedAssessmentReadiness post_kinchk() const noexcept { return TiedAssessmentReadiness::Pending; }
  private:
    struct Storage;
    explicit TiedSearchClassification(std::shared_ptr<const Storage> value) : storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
}
