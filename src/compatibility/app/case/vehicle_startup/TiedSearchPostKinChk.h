#pragma once
#include "TiedSearchClassification.h"
#include "lib_src/constraints/tied_shell/TiedPostKinChk.h"

namespace crash::cases::vehicle_startup {
inline constexpr const char* PostKinChkSourcePolicy =
    "openradioss_a62b27e_original_observed_post_kinchk_mesh_wall_v1";
enum class TiedPostKinChkPhase { ObservedKinetAfterKinChk };
enum class GeneratedRigidReadSet { OutsideObservedOriginalSlaves };
struct TiedPostKinChkReceipt {
    tied::UnresolvedBlock original_contact;
    std::uint64_t source_instance_id = 0;
    std::uint32_t native_interface_ordinal = 0;
    tied::SourceId replaced_wall_part = 0;
    native_search::KinChkProfile native_profile = native_search::KinChkProfile::Unspecified;
    GeneratedRigidReadSet generated_rigid_read_set = GeneratedRigidReadSet::OutsideObservedOriginalSlaves;
    std::size_t observed_slaves = 0;
    // Source/converted roles and the named whole-wall replacement prove these
    // consumed roles absent. This is not a generated global condition map.
    std::size_t effective_primitive_walls = 0, rbe2_roles = 0, rbe3_roles = 0, cyclic_roles = 0;
};
struct TiedPostKinChkLimits {
    std::size_t host_bytes = 512 * 1024 * 1024;
    native_search::KinChkLimits native;
};
struct TiedPostKinChkForecast {
    std::size_t retained_classification_reservation_bytes = 0;
    std::size_t input_staging_bytes = 0, receipt_payload_bytes = 0;
    native_search::KinChkForecast native;
    std::size_t total_host_bytes = 0;
};
// Immutable observation of KINET on the complete finalized original NSV.
// The classification handle supplies original source IDs/rows and unchanged
// CIN/PEN association. Global rigid registration, INIVCHK, physical coefficients
// and constraint/owner admission remain separate producer obligations.
class TiedSearchPostKinChk {
  public:
    static TiedPostKinChkForecast Forecast(const TiedSearchClassification&, TiedPostKinChkLimits = {});
    static TiedSearchPostKinChk Prepare(const TiedSearchClassification&, TiedPostKinChkLimits = {});
    TiedSearchPostKinChk(const TiedSearchPostKinChk&) noexcept = default;
    TiedSearchPostKinChk(TiedSearchPostKinChk&& other) noexcept : storage_(other.storage_) {}
    TiedSearchPostKinChk& operator=(const TiedSearchPostKinChk&) = delete;
    TiedSearchPostKinChk& operator=(TiedSearchPostKinChk&&) = delete;
    const TiedSearchClassification& classification() const noexcept;
    const TiedPostKinChkReceipt& receipt() const noexcept;
    const native_search::PostKinChkResult& result() const noexcept;
    const TiedPostKinChkForecast& forecast() const noexcept;
    TiedPostKinChkPhase phase() const noexcept { return TiedPostKinChkPhase::ObservedKinetAfterKinChk; }
  private:
    struct Storage;
    explicit TiedSearchPostKinChk(std::shared_ptr<const Storage> value) : storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
}
