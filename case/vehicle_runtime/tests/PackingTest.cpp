#include "../Packing.h"
#include "../Config.h"
#include "output/ArtifactIO.h"
#include "lib_utest/qualification/rigid_assembly_owner/Fixture.h"
namespace crash::cases::vehicle_runtime::test {
namespace {
SourceRoles Roles(const rigid_assembly_owner_test::Fixture& f) {
    SourceRoles roles;
    roles.node.resize(f.domain.node_count());
    for (auto node:f.shells.mapping()) roles.node[node] |= Shell;
    for (std::size_t c=0;c<f.springs.connection_count();++c)
        for (auto node:f.springs.connections()[c].global_node) roles.node[node] |= Type25Endpoint;
    for (const auto& group:f.binding.groups())
        for (std::size_t k=0;k<group.member_count;++k)
            roles.node[f.binding.members()[group.member_offset+k].domain_node] |=
                group.source_kind==tl::fea::RigidBindingSourceKind::Part ? Part : PlainRigid;
    for (std::size_t r=0;r<f.cin_model.rows().count;++r) {
        const auto& row=f.cin_model.rows().data[r];
        roles.node[row.secondary_domain_node] |= CinSecondary;
        for (auto node:row.master_domain_nodes) roles.node[node] |= CinMaster;
    }
    return roles;
}
}
TEST(VehicleRuntimePacking, RealDependentCoefficientsAndAbsentOrdinaryRotation) {
    rigid_assembly_owner_test::Fixture f(false,true,.002,true);
    const auto roles=Roles(f);
    const auto bytes=detail::PackingBytes(f.domain.node_count(),1u<<20);
    const auto packed=detail::PackOwner(f.ledger,f.binding,roles,{InitialSpeedMps,0,0},bytes);
    EXPECT_EQ(packed.capacity_bytes(),bytes);
    EXPECT_EQ(packed.rotation_present[f.zero_mass],1);
    EXPECT_EQ(packed.mass[f.zero_mass],0);
    EXPECT_EQ(packed.inverse_mass[f.zero_mass],0);
    EXPECT_EQ(packed.rotation_present[f.ordinary],0);
    EXPECT_EQ(packed.rotation_fixed[f.ordinary],0);
    EXPECT_EQ(packed.inverse_inertia[f.ordinary],0);
    for (std::size_t i=0;i<f.domain.node_count();++i) {
        EXPECT_EQ(output::Bits(packed.mass[i]),output::Bits(f.ledger.nodes()[i].coefficients.mass));
        EXPECT_EQ(output::Bits(packed.inertia[i]),output::Bits(f.ledger.nodes()[i].coefficients.isotropic_inertia));
        EXPECT_EQ(packed.velocity[3*i],InitialSpeedMps);
        EXPECT_EQ(packed.orientation[4*i],1);
        if (roles.node[i]&CinSecondary) {
            EXPECT_EQ(packed.inverse_mass[i],0);
            EXPECT_EQ(packed.inverse_inertia[i],0);
            EXPECT_EQ(packed.rotation_present[i],1);
        }
    }
}
TEST(VehicleRuntimePacking, LateRoleAndExactBudgetRejectionPreserveDestinationThenRetry) {
    rigid_assembly_owner_test::Fixture f(false,true,.002,true);
    auto roles=Roles(f);
    const auto bytes=detail::PackingBytes(f.domain.node_count(),1u<<20);
    auto destination=detail::PackOwner(f.ledger,f.binding,roles,{InitialSpeedMps,0,0},bytes);
    const auto old=destination.position;
    EXPECT_THROW(destination=detail::PackOwner(f.ledger,f.binding,roles,{InitialSpeedMps,0,0},bytes-1),std::runtime_error);
    roles.node.back() |= 128;
    EXPECT_THROW(destination=detail::PackOwner(f.ledger,f.binding,roles,{InitialSpeedMps,0,0},bytes),std::runtime_error);
    EXPECT_EQ(destination.position,old);
    roles.node.back() &= 127;
    destination=detail::PackOwner(f.ledger,f.binding,roles,{InitialSpeedMps,0,0},bytes);
    EXPECT_EQ(destination.position,old);
    roles.node[f.zero_mass] &= ~Part;
    EXPECT_THROW(detail::PackOwner(f.ledger,f.binding,roles,{InitialSpeedMps,0,0},bytes),std::runtime_error);
}
TEST(VehicleRuntimePacking, ExplicitInitialDescriptorAndLimits) {
    Config config;
    EXPECT_NO_THROW(detail::CheckConfig(config));
    EXPECT_DOUBLE_EQ(InitialSpeedMps,15.6464);
    config.reserved_step_s=0;
    EXPECT_THROW(detail::CheckConfig(config),std::runtime_error);
    config=Config{};
    config.configuration_id=0;
    EXPECT_THROW(detail::CheckConfig(config),std::runtime_error);
    EXPECT_THROW(detail::PackingBytes(tl::fea::MaxActiveNodalStateNodes+1,1u<<20),std::runtime_error);
}
TEST(VehicleRuntimePacking, ExplicitDevicePayloadCeilingKeepsDefaultAndRejectsOverCap) {
    Config config;
    EXPECT_EQ(config.limits.device_bytes, std::size_t{4} << 30);
    config.limits.device_bytes = std::size_t{5} << 30;
    EXPECT_NO_THROW(detail::CheckConfig(config));
    config.limits.device_bytes = Limits::maximum_device_bytes;
    EXPECT_NO_THROW(detail::CheckConfig(config));
    ++config.limits.device_bytes;
    EXPECT_THROW(detail::CheckConfig(config), std::runtime_error);
    config.limits.device_bytes = 0;
    EXPECT_THROW(detail::CheckConfig(config), std::runtime_error);
}
} // namespace crash::cases::vehicle_runtime::test
