#include "../source/PhysicalSelection.h"
#include <gtest/gtest.h>
namespace crash::cases::vehicle_run::test {
TEST(VehicleRunPhysicalSelection, CompletePoliciesStayPairedAndOnlyV5RequiresStructuralBeams) {
    using P=PhysicalProfile;
    const P profiles[]{P::RetainedShellAssembliesV1,P::ExtendedSolidsV4,P::VehicleSupportsV5};
    const std::size_t joints[]{38,40,44};
    for(unsigned i=0;i<3;++i) {
        const auto selected=detail::SelectPhysical(profiles[i]);
        EXPECT_EQ(selected.solids,modelio::physical_domain::detail::SolidPolicy(selected.domain));
        EXPECT_EQ(selected.domain,modelio::type45::detail::DomainPolicy(selected.joints));
        EXPECT_EQ(modelio::type45::detail::Required(selected.joints),joints[i]);
        EXPECT_EQ(selected.structural_beams,i==2); EXPECT_EQ(selected.extended,i!=0);
    }
    EXPECT_EQ(detail::SelectPhysical(P::VehicleSupportsV5).solids,modelio::solid_source::Policy::OriginalVehicleSupportsV5);
    auto accepted=detail::SelectPhysical(P::VehicleSupportsV5);
    EXPECT_THROW(accepted=detail::SelectPhysical(static_cast<P>(99)),std::invalid_argument);
    EXPECT_TRUE(accepted.structural_beams);
    EXPECT_EQ(modelio::type45::detail::Required(accepted.joints),44u);
}
} // namespace crash::cases::vehicle_run::test
