#pragma once
#include "InterfaceFields.h"
#include "TiedRemovalSource.h"
#include "lib_src/collision/RadiossType25InitialState.h"
namespace crash::cases::vehicle_native_contact::detail {
namespace initial = native::initial_source;
struct InitialModelForecast { std::size_t retained_bytes = 0; };
class InitialModel {
  public:
    InitialModel(InitialModel&&) noexcept = default;
    InitialModel& operator=(InitialModel&&) noexcept = default;
    InitialModel(const InitialModel&) = delete;
    InitialModel& operator=(const InitialModel&) = delete;
    static InitialModelForecast Preflight(const SourceInputs&, std::size_t byte_cap);
    static InitialModel Prepare(const SourceInputs&, const TiedRemovalSource&, std::size_t byte_cap);
    initial::Input input(const SourceInputs&, vehicle_dynamics::native_contact::Role,
                         const InterfaceFields&) const;
    const InitialModelForecast& forecast() const noexcept { return forecast_; }
  private:
    InitialModel() = default;
    std::vector<initial::EightSlotSolid> solids_;
    std::vector<initial::InterfaceIdentity> interfaces_;
    std::vector<tied_removal::Interface> tied_;
    InitialModelForecast forecast_;
};
// Shares the immutable SourceInputs and TiedRemovalSource backing. Their
// lifetimes cover host preparation and the later drained GeneralInitialize.
} // namespace crash::cases::vehicle_native_contact::detail
