#pragma once
#include "SourceAdmission.h"
#include "FieldPacking.h"
#include <optional>
namespace crash::cases::vehicle_native_contact::detail {
struct InterfaceFieldForecast {
    std::size_t starter_fields = 0, ready_fields = 0, scalar_vectors = 0, retained_bytes = 0;
};
// Case-private staging. SourceInputs remains immutable/alive through every
// borrowed view and General* call. These fields are before final initialization.
class InterfaceFields {
  public:
    static InterfaceFieldForecast Preflight(const SourceInputs&,
        vehicle_dynamics::native_contact::Role, FieldPackingLimits = {});
    static InterfaceFields Prepare(const SourceInputs&, vehicle_dynamics::native_contact::Role,
                                   FieldPackingLimits = {});
    InterfaceFields(InterfaceFields&&) noexcept = default;
    InterfaceFields& operator=(InterfaceFields&&) noexcept = default;
    InterfaceFields(const InterfaceFields&) = delete;
    InterfaceFields& operator=(const InterfaceFields&) = delete;
    lifecycle::SourceView starter_view() const noexcept { return starter_->view(); }
    lifecycle::SourceView runtime_view() const noexcept { return ready_ ? ready_->view() : starter_->view(); }
    tl::util::ConstView<double> main_search_gaps() const noexcept { return {main_search_gaps_.data(), main_search_gaps_.size()}; }
    const InterfaceFieldForecast& forecast() const noexcept { return forecast_; }
  private:
    InterfaceFields() = default;
    InterfaceFieldForecast forecast_;
    std::vector<double> secondary_coefficients_, main_search_gaps_;
    std::optional<FieldPacking> starter_, ready_;
};
} // namespace crash::cases::vehicle_native_contact::detail
