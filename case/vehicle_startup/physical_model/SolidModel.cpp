#include "Internal.h"

namespace crash::cases::vehicle_startup::physical_model::detail {
void PrepareSolids(const modelio::solid_source::VehicleSolidSource& source, const fe::NodalNodeDomain& domain,
                   std::size_t cap, fe::solids::Model& model) {
    const auto& data = source.data();
    namespace src = modelio::solid_source;
    std::vector<fe::solids::Input18> adhesive;
    std::vector<fe::solids::Input24> brick;
    std::vector<fe::solids::Input6z> wedge;
    std::vector<fe::solids::Input18Law44> rear;
    adhesive.reserve(data.solid18.size()); brick.reserve(data.solid24.size()); wedge.reserve(data.solid6z.size());
    rear.reserve(data.solid18_law44.size());
    for (const auto& row : data.rows) {
        const auto& part = data.parts.at(row.part_index);
        if (row.family == src::Family::Solid18) {
            Require(row.reference_index == adhesive.size(), "Original solid18 family order changed");
            adhesive.push_back({data.solid18.at(row.reference_index), part.law36});
        } else if (row.family == src::Family::Solid24) {
            Require(row.reference_index == brick.size(), "Original HEPH family order changed");
            brick.push_back({data.solid24.at(row.reference_index), part.law42});
        } else if (row.family == src::Family::Solid18Law44) {
            Require(row.reference_index == rear.size(), "Original rear LAW44 family order changed");
            rear.push_back({data.solid18_law44.at(row.reference_index), part.law44});
        } else {
            Require(row.family == src::Family::Solid6z && row.reference_index == wedge.size(),
                    "Original S6Z family or source order changed");
            wedge.push_back({data.solid6z.at(row.reference_index), part.law42, data.wedge_force_profile});
        }
    }
    fe::solids::ModelLimits limits; limits.max_host_bytes = cap;
    fe::solids::ModelInput input{domain.source_instance_id(),
        {adhesive.data(), adhesive.size()}, {brick.data(), brick.size()}, {wedge.data(), wedge.size()}};
    input.solid18_law44 = {rear.data(), rear.size()};
    if (!rear.empty()) input.profile = fe::solids::ModelProfile::ExtendedLaw44Law90;
    const auto report = model.Initialize(domain, input, limits);
    Require(bool(report), report.message);
}
} // namespace crash::cases::vehicle_startup::physical_model::detail
