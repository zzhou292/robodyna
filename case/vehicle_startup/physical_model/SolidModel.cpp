#include "Internal.h"

namespace crash::cases::vehicle_startup::physical_model::detail {
void PrepareSolids(const modelio::solid_source::VehicleSolidSource& source, const fe::NodalNodeDomain& domain,
                   std::size_t cap, fe::solids::Model& model) {
    const auto& data = source.data();
    namespace src = modelio::solid_source;
    std::vector<fe::solids::Input18> adhesive;
    std::vector<fe::solids::Input24> brick;
    std::vector<fe::solids::Input6z> wedge;
    adhesive.reserve(data.solid18.size()); brick.reserve(data.solid24.size()); wedge.reserve(data.solid6z.size());
    for (const auto& row : data.rows) {
        const auto& part = data.parts.at(row.part_index);
        if (row.family == src::Family::Solid18) {
            Require(row.reference_index == adhesive.size(), "Original solid18 family order changed");
            adhesive.push_back({data.solid18.at(row.reference_index), part.law36});
        } else if (row.family == src::Family::Solid24) {
            Require(row.reference_index == brick.size(), "Original HEPH family order changed");
            brick.push_back({data.solid24.at(row.reference_index), part.law42});
        } else {
            Require(row.family == src::Family::Solid6z && row.reference_index == wedge.size(),
                    "Original S6Z family or source order changed");
            wedge.push_back({data.solid6z.at(row.reference_index), part.law42, data.wedge_force_profile});
        }
    }
    fe::solids::ModelLimits limits; limits.max_host_bytes = cap;
    const auto report = model.Initialize(domain, {domain.source_instance_id(),
        {adhesive.data(), adhesive.size()}, {brick.data(), brick.size()}, {wedge.data(), wedge.size()}}, limits);
    Require(bool(report), report.message);
}
} // namespace crash::cases::vehicle_startup::physical_model::detail
