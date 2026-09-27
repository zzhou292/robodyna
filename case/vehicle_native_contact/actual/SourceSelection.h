#pragma once
#include "../source/OriginalSources.h"
#include "../VehicleContactStartup.h"
#include <optional>
namespace crash::cases::vehicle_native_contact::test {
// Owns the explicit production graph while the runner borrows SourceInputs.
// No packet artifact means the previously qualified V5 fixture is selected.
class SourceSelection {
  public:
    explicit SourceSelection(Config&);
    detail::SourceInputs Prepare();
    bool native_v6()const noexcept{return artifact_.has_value();}
    std::size_t extra_retained_bytes()const noexcept;
  private:
    std::optional<modelio::solid_control_packets::Artifact> artifact_;
    std::optional<source::OriginalSources> source_;
};
} // namespace crash::cases::vehicle_native_contact::test
