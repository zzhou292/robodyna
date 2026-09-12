#include "Internal.h"

namespace crash::cases::vehicle_startup::physical_model::detail {
void PrepareStructuralBeams(const modelio::beam18::Source& source, const fe::NodalNodeDomain& domain,
                            std::size_t cap, fe::beam18::Model& model) {
    const auto& data = source.data();
    Require(data.policy == modelio::beam18::Policy::OriginalCircularFourPointLaw44V1 && data.rows.size() == 142,
            "Unsupported original structural beam source");
    std::vector<fe::beam18::ParentInput> parents;
    parents.reserve(data.rows.size());
    Require(parents.capacity() == data.rows.size(), "Structural beam input capacity exceeds preflight");
    for (const auto& row : data.rows) {
        const auto& part = data.parts.at(row.part_index);
        Require(part.id == row.part_id, "Structural beam source part association changed");
        parents.push_back({row.reference, part.material});
    }
    fe::beam18::ModelLimits limits;
    limits.max_host_bytes = cap;
    const fe::beam18::ModelInput input{domain.source_instance_id(), {parents.data(), parents.size()},
        fe::beam18::ModelProfile::CircularFourPointLaw44V1};
    const auto report = model.Initialize(domain, input, limits);
    Require(bool(report), report.message);
}
} // namespace crash::cases::vehicle_startup::physical_model::detail
