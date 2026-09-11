#pragma once
#include "TiedSearchAssessment.h"
#include "lib_src/constraints/tied_shell/search/TiedSearchFinalization.h"
namespace crash::cases::vehicle_startup {
inline constexpr const char* TiedFinalizationSourcePolicy =
    "openradioss_a62b27e_serial_first_ordinary_type2_level28_fresh_v1";
enum class TiedFinalizationPhase { FreshPhysicalCoefficientsBeforeIniend };
struct TiedFinalizationReceipt {
    std::size_t contact_source = 0;
    std::size_t source_file_count = 0, source_contact_count = 0;
    std::size_t type2_count = 0, type2_ordinal = 0;
    std::size_t unique_original_slaves = 0;
    int is1 = 0, level = 0, ignore = 0, projection = 0;
    std::size_t prior_connection_count = 0, multiple_connection_count = 0;
    double search_distance = 0;
    TiedFinalizationPhase phase = TiedFinalizationPhase::FreshPhysicalCoefficientsBeforeIniend;
};
struct TiedFinalizationLimits {
    std::size_t host_bytes = 512 * 1024 * 1024;
    std::size_t metadata_bytes = 8 * 1024 * 1024;
    std::size_t source_files = 16, source_blocks = 16384;
    native_search::FinalizationLimits native;
};
struct TiedFinalizationForecast {
    std::size_t retained_source_bytes = 0, retained_assessment_bytes = 0;
    std::size_t input_staging_bytes = 0, metadata_reservation_bytes = 0;
    std::size_t finalizer_reservation_bytes = 0, fixed_bytes = 0, total_host_bytes = 0;
};
// Immutable source-bound finalization of one complete geometric assessment.
// Original IRECT and every original NSV disposition remain accessible; no
// classifier masks, physical M/J, runtime owner, or solver clock are produced.
class TiedSearchFinalized {
  public:
    static TiedFinalizationForecast Forecast(const TiedSearchAssessment&, TiedFinalizationLimits = {});
    static TiedSearchFinalized Prepare(const TiedSearchAssessment&, TiedFinalizationLimits = {});
    TiedSearchFinalized(const TiedSearchFinalized&) noexcept = default;
    TiedSearchFinalized(TiedSearchFinalized&& other) noexcept : storage_(other.storage_) {}
    TiedSearchFinalized& operator=(const TiedSearchFinalized&) = delete;
    TiedSearchFinalized& operator=(TiedSearchFinalized&&) = delete;
    const TiedSearchAssessment& assessment() const noexcept;
    const native_search::FinalizationMaps& data() const noexcept;
    const TiedFinalizationReceipt& receipt() const noexcept;
    const TiedFinalizationForecast& forecast() const noexcept;
    const tied::Node& secondary(std::size_t compact_row) const;
    const tied::Element& selected_master(std::size_t compact_row) const;
    TiedAssessmentReadiness classification() const noexcept { return TiedAssessmentReadiness::Pending; }
  private:
    struct Storage;
    explicit TiedSearchFinalized(std::shared_ptr<const Storage> value) : storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
}
