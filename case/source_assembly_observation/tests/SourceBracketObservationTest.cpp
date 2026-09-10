#include "case/source_assembly_observation/SourceAssemblyQephSpin.h"
#include "ForceStageFixture.h"
#include "case/source_assembly/tests/SourceBracketTestSupport.h"

namespace crash::cases::source_assembly_observation::test {
TEST(SourceBracketObservation, CombinedMassAndInertiaKickUsesExplicitConnectorPartitions) {
    Fixture f(source_assembly::test::BracketBindings());
    const auto n=f.bindings.connectors()->connections()[0].global_node[1];
    const auto& shell=f.bindings.shells().nodes()[n].native;
    const long double mass=static_cast<long double>(shell.mass)+.0005L;
    const long double inertia=static_cast<long double>(shell.isotropic_inertia)+5e-9L;
    f.next.v[3*n]=8.25;f.next.w[3*n+2]=2;
    f.force[3*n]=static_cast<double>(mass*.25L/(.5L*H));
    f.couple[3*n+2]=static_cast<double>(inertia*2/(.5L*H));
    Summary result;const auto report=ObserveInterval(f.input(),&result);ASSERT_TRUE(report)<<report.message;
    Near(result.native_delta,.5L*mass*(8.25L*8.25L-64)+2*inertia);
    Near(result.after.connector.translation,.5L*.0005L*(64+8.25L*8.25L));
    Near(result.after.connector.rotation,2*5e-9L);
    EXPECT_EQ(result.replacement_delta,0);EXPECT_LE(std::abs(result.native_residual),result.roundoff_budget);
    EXPECT_LT(std::abs(result.after.ordinary.inertia_partition_residual),1e-18);
    const auto saved=Bytes(result);auto corrupted=f.input();corrupted.kinetic.connector_rotation=0;
    EXPECT_EQ(ObserveInterval(corrupted,&result).status,Status::KineticMismatch);EXPECT_EQ(Bytes(result),saved);
    f.couple[3*n+2]=static_cast<double>(static_cast<long double>(shell.isotropic_inertia)*2/(.5L*H));
    EXPECT_EQ(ObserveInterval(f.input(),&result).status,Status::KickMismatch);EXPECT_EQ(Bytes(result),saved);
}
TEST(SourceBracketObservation, ForceStageOrdinaryAccelerationUsesTotalJWithoutChangingGroupReplacement) {
    ForceFixture f(source_assembly::test::BracketBindings());f.Later();
    ForceStageSummary result;const auto report=ObserveForceStage(f.force_input(),&result);ASSERT_TRUE(report)<<report.message;
    long double translation=0,rotation=0;
    for(unsigned endpoint=0;endpoint<2;++endpoint) {
        const auto n=f.bindings.connectors()->connections()[0].global_node[endpoint];
        for(unsigned a=0;a<3;++a) {
            // Independent scalar collocation from supplied pre-kick samples,
            // with the same declared force-stage DT1/2, not kicked velocity.
            const double v=f.old.v[3*n+a]+.5*H*f.acceleration.v[3*n+a];
            const double w=f.old.w[3*n+a]+.5*H*f.acceleration.w[3*n+a];
            translation+=.5L*.0005L*v*v;rotation+=.5L*5e-9L*w*w;
        }
    }
    Near(result.connector.translation,translation);Near(result.connector.rotation,rotation);
    EXPECT_LT(std::abs(result.ordinary.inertia_partition_residual),1e-14);
    EXPECT_EQ(result.native_total,result.ordinary.total+result.grouped_members.total);
    EXPECT_EQ(result.effective_total,result.ordinary.total+result.groups.total);
    const auto saved=Bytes(result);const auto n=f.bindings.connectors()->connections()[0].global_node[1];
    f.acceleration.w[3*n+2]=std::numeric_limits<double>::infinity();
    EXPECT_EQ(ObserveForceStage(f.force_input(),&result).status,Status::NonfiniteResult);EXPECT_EQ(Bytes(result),saved);
}
TEST(SourceBracketObservation, ShellOnlySpinAttributionRejectsConnectorEndpointsBeforeSampling) {
    const auto bindings=source_assembly::test::BracketBindings();
    EXPECT_FALSE(CheckQephSpinSource(bindings,2181504));
    EXPECT_FALSE(CheckQephSpinSource(bindings,2204838));
    EXPECT_TRUE(CheckQephSpinSource(bindings,2181592));
}
} // namespace crash::cases::source_assembly_observation::test
