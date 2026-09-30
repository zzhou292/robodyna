#pragma once
#include "modelio/tied_shell/TiedShellSearchGeometry.h"
#include "lib_src/constraints/tied_shell/search/TiedSearchDriver.h"
#include <stdexcept>

namespace crash::cases::vehicle_startup {
namespace tied = modelio::tied_shell;
namespace native_search = tl::constraints::tied_shell;
enum class TiedAssessmentReadiness { Pending };
struct TiedAssessmentLimits {
    std::size_t host_bytes = 512 * 1024 * 1024;
    native_search::SearchDriverLimits driver = [] {
        native_search::SearchDriverLimits value;
        value.max_host_bytes = 128 * 1024 * 1024;
        return value;
    }();
};
struct TiedAssessmentForecast {
    std::size_t source_payload_bytes = 0;
    std::size_t input_staging_bytes = 0;
    std::size_t fixed_bytes = 0;
    // Reservation: driver charges actual CUB scratch and bounded pair capacity
    // within these limits before allocation. The retained result reports it.
    std::size_t driver_host_reservation_bytes = 0;
    std::size_t device_limit_bytes = 0;
    std::size_t total_host_bytes = 0;
};
class TiedAssessmentError : public std::runtime_error {
  public:
    explicit TiedAssessmentError(native_search::SearchDriverReport report,
                                 tied::SourceId node, tied::SourceId master)
        : std::runtime_error(report.message), report(report), source_node_id(node), source_master_id(master) {}
    const native_search::SearchDriverReport report;
    const tied::SourceId source_node_id, source_master_id;
};
// Immutable complete geometric assessment. It retains the authenticated source
// backing and original IRECT patch association even when equivalent material
// witnesses supplied its thickness. No finalization/classification or mechanics
// admission, no master substitution after a selected force patch is rejected.
class TiedSearchAssessment {
  public:
    static TiedAssessmentForecast Forecast(const tied::TiedShellSearchGeometry&, TiedAssessmentLimits = {});
    static TiedSearchAssessment Prepare(const tied::TiedShellSearchGeometry&, TiedAssessmentLimits = {});
    TiedSearchAssessment(const TiedSearchAssessment&) noexcept = default;
    TiedSearchAssessment(TiedSearchAssessment&& other) noexcept : data_(other.data_) {}
    TiedSearchAssessment& operator=(const TiedSearchAssessment&) = delete;
    TiedSearchAssessment& operator=(TiedSearchAssessment&&) = delete;
    const tied::TiedShellSearchGeometry& geometry() const noexcept;
    const native_search::SearchDriverResult& result() const noexcept;
    const TiedAssessmentForecast& forecast() const noexcept;
    const tied::Node& secondary(std::size_t nsv_row) const;
    // Null for unmatched; otherwise the original declared mechanical patch.
    const tied::Element* selected_master(std::size_t nsv_row) const;
    tied::SourceId selected_part_id(std::size_t nsv_row) const;
    TiedAssessmentReadiness finalization() const noexcept { return TiedAssessmentReadiness::Pending; }
    TiedAssessmentReadiness classification() const noexcept { return TiedAssessmentReadiness::Pending; }
  private:
    struct Storage;
    explicit TiedSearchAssessment(std::shared_ptr<const Storage> data) : data_(std::move(data)) {}
    std::shared_ptr<const Storage> data_;
};
} // namespace crash::cases::vehicle_startup
