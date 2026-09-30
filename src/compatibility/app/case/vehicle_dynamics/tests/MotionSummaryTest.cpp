#include "../MotionSummary.h"
#include "lib_utest/qualification/rigid_assembly_owner/Fixture.h"
#include <limits>
namespace crash::cases::vehicle_dynamics::test {
TEST(VehiclePhysicalMotion, EveryPhysicalFieldAndLateComponentHaveAnIndependentTranslationOracle) {
    rigid_assembly_owner_test::Fixture fixture(false,true,.002,true);
    const auto& ledger=fixture.ledger;Fields fields(ledger.nodes().size());
    const double speed=16,time=.25;
    for(std::size_t i=0;i<ledger.nodes().size();++i) {
        const auto p=ledger.domain()->nodes()[i].position;
        fields.position[3*i]=p.x+4;fields.position[3*i+1]=p.y;fields.position[3*i+2]=p.z;
        fields.velocity[3*i]=16;fields.orientation[4*i]=1;
    }
    auto observed=ObserveUniformMotion(ledger,fields,speed,time);
    EXPECT_EQ(observed.nodes,ledger.nodes().size());
    EXPECT_EQ(observed.maximum_position_error,0);EXPECT_EQ(observed.maximum_velocity_error,0);
    fields.position.back()+=.25;fields.velocity.back()=.5;fields.orientation.back()=.125;fields.spin.back()=-.75;
    observed=ObserveUniformMotion(ledger,fields,speed,time);
    EXPECT_EQ(observed.maximum_position_error,.25);EXPECT_EQ(observed.maximum_velocity_error,.5);
    EXPECT_EQ(observed.maximum_orientation_error,.125);EXPECT_EQ(observed.maximum_spin,.75);
}
TEST(VehiclePhysicalMotion, NonfiniteFinalFieldsAndIncompleteSnapshotsAreRejected) {
    rigid_assembly_owner_test::Fixture fixture;Fields fields(fixture.ledger.nodes().size());
    fields.spin.back()=std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(ObserveUniformMotion(fixture.ledger,fields,16,0),std::runtime_error);
    fields.spin.back()=0;fields.orientation.pop_back();
    EXPECT_THROW(ObserveUniformMotion(fixture.ledger,fields,16,0),std::runtime_error);
}
} // namespace crash::cases::vehicle_dynamics::test
