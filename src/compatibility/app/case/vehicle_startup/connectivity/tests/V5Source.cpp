#include "V5Fixture.h"
#include "../../physical_attachments/tests/PreparePost.h"

namespace crash::cases::vehicle_startup::connectivity::test {
const VehicleConnectivity& ActualV5() {
    static const auto value = [] {
        namespace fixture = physical_model::supports_test;
        const auto post = physical_attachments::test::PreparePost(fixture::Scope(),fixture::Inputs().member);
        const auto source = VehiclePhysicalAttachments::Prepare(fixture::Model(),post);
        const auto& joints = fixture::Joints();
        const auto forecast = VehicleConnectivity::PreflightWithJoints(source,joints);
        std::cout << "VehicleConnectivity V5 nodes=" << forecast.extents.nodes <<
            " relations=" << forecast.extents.relations << " slots=" << forecast.extents.slots <<
            " owned=" << forecast.owned_bytes << " total=" << forecast.total_bytes << std::endl;
        return VehicleConnectivity::PrepareWithJoints(source,joints);
    }();
    return value; // Local source/post handles have retired; the graph retains exact authorities.
}
} // namespace crash::cases::vehicle_startup::connectivity::test
