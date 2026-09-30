#pragma once
#include "VehiclePointMassSource.h"

namespace crash::modelio::point_mass::detail {
struct MappedRecords {
    std::vector<tl::fea::ElementMassSource> retained;
    std::vector<Disposition> dispositions;
};
// Typed value mapping; the public factory supplies authenticated source cards
// and the separately checked canonical domain. This helper grants neither.
MappedRecords Map(const std::vector<physical_scope::rigid::point_mass::Record>&,
                  const tl::fea::NodalNodeDomain&, std::size_t record_cap);
} // namespace crash::modelio::point_mass::detail
