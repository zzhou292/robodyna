#pragma once
#include "case/vehicle_startup/physical_attachments/VehiclePhysicalAttachments.h"

namespace crash::cases::vehicle_runtime {
enum SourceRole : std::uint8_t {
    Shell = 1, Part = 2, PlainRigid = 4, CinSecondary = 8, CinMaster = 16,
    Type13Endpoint = 32, Type25Endpoint = 64, Beam18Endpoint = 128
};
struct SourceRoles {
    std::vector<std::uint8_t> node;
    std::size_t capacity_bytes() const noexcept { return node.capacity(); }
};
// Source incidence only. Does not admit constraints, reciprocal coefficients,
// rotational DOFs or a CUDA owner. N3 never receives an endpoint role.
SourceRoles ResolveSourceRoles(const vehicle_startup::physical_attachments::VehiclePhysicalAttachments&,
                              std::size_t max_nodes = 524288);
} // namespace crash::cases::vehicle_runtime
