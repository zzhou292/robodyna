#pragma once
#include "SourcePartElasticTestSupport.h"
#include "SourcePartElasticWallInternal.h"
#include "case/source_part_wall/SourcePartWallCheckFixture.h"
#include "case/source_part_wall/SourcePartWallResultCheck.h"

namespace crash::cases::source_part_elastic::test {
namespace wall_test=source_part_wall::check;
inline constexpr unsigned WallOnsetPrefix=8448;
inline void HostWall(const SourcePartElasticCase& run,const Snapshot& state,std::uint64_t base_epoch,
    std::uint64_t attempt,wall_contact::NodalWallResult& output) {
    const auto& setup=*run.wall_setup();const auto& settings=*setup.settings();
    const wall_contact::NodalWallConfig law{setup.placed_wall()->geometry()->wall_x(),
        setup.certificate()->stiffness_per_area,settings.penetration_cap,settings.parent_force_error,settings.parent_energy_error};
    const wall_contact::LumpedTranslationMassView mass{setup.inverse_mass(),setup.translation_fixed_bits(),NodeCount,
        base_epoch,wall_contact::TranslationMassModel::kIsotropicLumped};
    ASSERT_EQ(wall_contact::EvaluateNodalWallContact(*setup.source_geometry()->weights(),
        {state.position.data(),NodeCount,3,1},{state.velocity.data(),NodeCount,3,1},mass,law,attempt,&output).status,
        wall_contact::NodalWallStatus::Ok);
}
inline void HostForce(const wall_contact::NodalWallResult& result,std::array<long double,3*NodeCount>& force) {
    force.fill(0);
    for(unsigned n=0;n<NodeCount;++n) {
        ASSERT_EQ(result.nodes[n].node,n);
        force[3*n]=result.nodes[n].force_world.x;
        force[3*n+1]=result.nodes[n].force_world.y;
        force[3*n+2]=result.nodes[n].force_world.z;
    }
}
inline void CertifiedAgreement(wall_contact::Q4CertifiedIntegral actual,wall_contact::Q4CertifiedIntegral expected) {
    ASSERT_TRUE(wall_contact::nodal_wall_detail::Certificate(actual));
    ASSERT_TRUE(wall_contact::nodal_wall_detail::Certificate(expected));
    EXPECT_LE(actual.lower,expected.upper); EXPECT_LE(expected.lower,actual.upper);
    EXPECT_LE(std::abs(actual.value-expected.value),actual.error+expected.error);
}
inline void HostAgreement(const wall_contact::NodalWallDeviceResults& actual,const wall_contact::NodalWallResult& expected) {
    CertifiedAgreement(actual.diagnostics.resultant,expected.resultant);
    CertifiedAgreement(actual.diagnostics.potential,expected.potential);
    for(unsigned n=0;n<NodeCount;++n) {
        SCOPED_TRACE(n);
        EXPECT_EQ(actual.nodes[n].node,expected.nodes[n].node);
        CertifiedAgreement(actual.nodes[n].force,expected.nodes[n].force);
        CertifiedAgreement(actual.nodes[n].potential,expected.nodes[n].potential);
    }
    for(unsigned e=0;e<source::ParentCount;++e) {
        SCOPED_TRACE(e);
        EXPECT_EQ(actual.parents[e].parent_element_id,expected.parents[e].parent_element_id);
        EXPECT_EQ(actual.parents[e].family,expected.parents[e].family);
        CertifiedAgreement(actual.parents[e].resultant,expected.parents[e].resultant);
        CertifiedAgreement(actual.parents[e].potential,expected.parents[e].potential);
        for(unsigned n=0;n<expected.parents[e].arity;++n)
            CertifiedAgreement(actual.parents[e].force[n],expected.parents[e].force[n]);
    }
}
inline void SameWallMetrics(const WallMetrics& a,const WallMetrics& b) {
    EXPECT_EQ(a.wall_kick_impulse,b.wall_kick_impulse); EXPECT_EQ(a.wall_kick_impulse_error,b.wall_kick_impulse_error);
    EXPECT_EQ(a.carried_momentum_residual,b.carried_momentum_residual); EXPECT_EQ(a.carried_momentum_allowance,b.carried_momentum_allowance);
    EXPECT_EQ(a.carried_angular_momentum,b.carried_angular_momentum); EXPECT_EQ(a.wall_kick_moment,b.wall_kick_moment);
    EXPECT_EQ(a.wall_kick_moment_error,b.wall_kick_moment_error);
    EXPECT_EQ(a.synchronized_kinetic_uncertainty,b.synchronized_kinetic_uncertainty);
    EXPECT_EQ(a.physical_energy_uncertainty,b.physical_energy_uncertainty); EXPECT_EQ(a.energy_allowance,b.energy_allowance);
    EXPECT_EQ(a.first_contact_epoch,b.first_contact_epoch); EXPECT_EQ(a.last_contact_epoch,b.last_contact_epoch);
    EXPECT_EQ(a.contact_intervals,b.contact_intervals); EXPECT_EQ(a.active_nodes,b.active_nodes);
}
// Called once at the actual onset checkpoint; no repeated long prefix for each
// fault. Save an uncommitted clean next candidate, inject late failures, then
// compare the accepted retry with those exact clean fields/results.
void CheckWallRollbackAndRetry(SourcePartElasticCase&,NativeSequence&);
} // namespace crash::cases::source_part_elastic::test
