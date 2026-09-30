#pragma once
#include "VehicleContactStartup.h"
#include "activity/Declaration.h"
namespace crash::cases::vehicle_native_contact {
struct VehicleContactStartup::Data {
    Data(const OwnerSource& o, const SelfSource& s, const WallSource& w, const ControlsSource& c, Config options)
        : owner(o), self(s), wall(w), controls(c), config(std::move(options)),
          physical_source(vehicle_runtime::Source::WithEnvironment(owner)) {}
    OwnerSource owner;
    SelfSource self;
    WallSource wall;
    ControlsSource controls;
    Config config;
    vehicle_runtime::Source physical_source;
    Forecast forecast;
    std::optional<activity::Declaration> activity_declaration;
    std::optional<TiedRemovalSource> tied;
    std::optional<detail::InitialModel> model;
    std::array<std::optional<detail::InterfaceFields>, 2> fields;
    std::array<n::initial_source::PreparedSource, 2> prepared;
    detail::SourceInputs sources() const { return {owner, self, wall, controls}; }
    n::TransactionConfig ContactConfig(Role) const;
    n::initial_source::Input InitialInput(Role) const;
    n::MixedMovingMainSource Self() const;
    n::FixedMainSource Wall() const;
    std::uint64_t TopologyGeneration(Role) const;
    void ForecastPreparation();
    void CompleteForecast();
};
namespace detail {
std::size_t RoleIndex(Role);
std::size_t AddBytes(std::size_t, std::size_t);
void CheckConfig(const Config&, const ControlsSource&);
}
} // namespace crash::cases::vehicle_native_contact
