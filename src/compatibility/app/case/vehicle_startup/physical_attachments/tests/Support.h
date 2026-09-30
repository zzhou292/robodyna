#pragma once
#include "../VehiclePhysicalAttachments.h"
#include "../../physical_model/tests/Support.h"

namespace crash::cases::vehicle_startup::physical_attachments::test {
const TiedSearchPostKinChk& Post();
inline const VehiclePhysicalAttachments& Actual() {
    static const auto value = [] {
        const auto& physical = physical_model::test::Actual();
        std::cout << "VehiclePhysicalAttachments complete forecast=" <<
            VehiclePhysicalAttachments::Preflight(physical, Post()).total_bytes << std::endl;
        return VehiclePhysicalAttachments::Prepare(physical, Post());
    }();
    return value;
}
} // namespace crash::cases::vehicle_startup::physical_attachments::test
