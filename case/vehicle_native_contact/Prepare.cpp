#include "Storage.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_native_contact {
Forecast VehicleContactStartup::ForecastPreparation(const OwnerSource& owner, const SelfSource& self,
                                                      const WallSource& wall, const ControlsSource& controls,
                                                      Config config) {
    Data data(owner, self, wall, controls, std::move(config));
    data.ForecastPreparation();
    return data.forecast;
}
VehicleContactStartup VehicleContactStartup::Prepare(const OwnerSource& owner, const SelfSource& self,
                                                     const WallSource& wall, const ControlsSource& controls,
                                                     Config config) {
    auto next = std::make_shared<Data>(owner, self, wall, controls, std::move(config));
    next->ForecastPreparation();
    const auto input = next->sources();
    next->tied.emplace(TiedRemovalSource::Prepare(owner, controls, next->config.tied));
    next->model.emplace(detail::InitialModel::Prepare(input, *next->tied, next->config.model_staging_bytes));
    for (const auto& entry : next->forecast.sources.interfaces) {
        const auto i = detail::RoleIndex(entry.role);
        next->fields[i].emplace(detail::InterfaceFields::Prepare(input, entry.role, next->config.fields));
        const auto source = next->InitialInput(entry.role);
        const auto report = n::initial_source::PrepareSource(source, next->config.initialization[i], next->prepared[i]);
        if (report.status != n::initial_source::Status::Ok)
            throw PreparationError(entry.role == Role::Self ? "Self initial source" : "Wall initial source", report);
    }
    for (const auto& entry : next->forecast.sources.interfaces) {
        const auto i = detail::RoleIndex(entry.role);
        const auto law = next->ContactConfig(entry.role);
        n::TransactionReport report;
        if (entry.role == Role::Self)
            report = n::Transaction::GeneralPreflight(law, next->Self(), next->prepared[i], owner.physical(),
                next->forecast.contact[i], next->config.transaction[i]);
        else
            report = n::Transaction::GeneralPreflight(law, next->Wall(), wall.fixed_ready(), next->prepared[i],
                owner.physical(), next->forecast.contact[i], next->config.transaction[i]);
        if (report.status != n::TransactionStatus::Ok)
            throw PreparationError(entry.role == Role::Self ? "Self runtime preflight" : "Wall runtime preflight", report);
    }
    next->CompleteForecast();
    return VehicleContactStartup(std::move(next));
}
const Forecast& VehicleContactStartup::forecast() const noexcept { return data_->forecast; }
const Config& VehicleContactStartup::config() const noexcept { return data_->config; }
const OwnerSource& VehicleContactStartup::owner_source() const noexcept { return data_->owner; }
const SelfSource& VehicleContactStartup::self_source() const noexcept { return data_->self; }
const WallSource& VehicleContactStartup::wall_source() const noexcept { return data_->wall; }
const ControlsSource& VehicleContactStartup::controls_source() const noexcept { return data_->controls; }
tl::util::ConstView<detail::OrderedInterface> VehicleContactStartup::interface_order() const noexcept {
    return {data_->forecast.sources.interfaces.data(), data_->forecast.sources.interfaces.size()};
}
n::initial_source::Input VehicleContactStartup::source_input(Role role) const { return data_->InitialInput(role); }
const n::initial_source::PreparedSource& VehicleContactStartup::prepared_source(Role role) const {
    return data_->prepared[detail::RoleIndex(role)];
}
} // namespace crash::cases::vehicle_native_contact
