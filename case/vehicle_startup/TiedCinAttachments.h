#pragma once
#include "TiedSearchPostKinChk.h"
#include "lib_src/constraints/tied_shell/TiedCinAttachmentModel.h"
#include <stdexcept>
#include <string>

namespace crash::cases::vehicle_startup {
class TiedCinAttachmentError : public std::runtime_error {
  public:
    TiedCinAttachmentError(native_search::CinAttachmentReport value,tied::SourceId secondary,tied::SourceId master)
        : std::runtime_error("CIN attachment reference/domain mapping rejected at row " + std::to_string(value.row)),
          report(value),source_secondary_id(secondary),source_master_id(master) {}
    const native_search::CinAttachmentReport report;
    const tied::SourceId source_secondary_id,source_master_id;
};
struct TiedCinAttachmentLimits {
    std::size_t host_bytes = 512 * 1024 * 1024;
    native_search::CinAttachmentLimits native;
};
struct TiedCinAttachmentForecast {
    std::size_t post_kinchk_reservation_bytes = 0;
    std::size_t input_staging_bytes = 0, fixed_bytes = 0;
    native_search::CinAttachmentForecast model;
    std::size_t total_host_bytes = 0;
};
// Complete original CIN attachment-to-declared-node map. The supplied physical
// domain may include other nodes; this proves only that every required original
// tied node and coordinate is represented. No additional participant/DOF or
// full vehicle source coverage is established by this immutable map.
class TiedCinAttachments {
  public:
    static TiedCinAttachmentForecast Forecast(const TiedSearchPostKinChk&,const tl::fea::NodalNodeDomain&,
                                              TiedCinAttachmentLimits = {});
    static TiedCinAttachments Prepare(const TiedSearchPostKinChk&,const tl::fea::NodalNodeDomain&,
                                      TiedCinAttachmentLimits = {});
    TiedCinAttachments(const TiedCinAttachments&) noexcept = default;
    TiedCinAttachments(TiedCinAttachments&& other) noexcept : storage_(other.storage_) {}
    TiedCinAttachments& operator=(const TiedCinAttachments&) = delete;
    TiedCinAttachments& operator=(TiedCinAttachments&&) = delete;
    const TiedSearchPostKinChk& post_kinchk() const noexcept;
    const native_search::TiedCinAttachmentModel& model() const noexcept;
    const TiedCinAttachmentForecast& forecast() const noexcept;
  private:
    struct Storage;
    explicit TiedCinAttachments(std::shared_ptr<const Storage> value) : storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
}
