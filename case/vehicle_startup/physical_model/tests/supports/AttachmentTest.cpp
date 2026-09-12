#include "Support.h"
#include "case/vehicle_startup/physical_attachments/tests/PreparePost.h"
#include "case/vehicle_runtime/Packing.h"
namespace crash::cases::vehicle_startup::physical_model::supports_test {
TEST(VehicleSupportsPhysicalAttachments, ActualCinAndAllEndpointRolesPackNativeCoefficientsWithoutAnOwner) {
    namespace attach=physical_attachments;
    namespace runtime=vehicle_runtime;
    const auto post=attach::test::PreparePost(Scope(),Inputs().member);
    const auto source=attach::VehiclePhysicalAttachments::Prepare(Model(),post);
    EXPECT_TRUE(source.attachments().model().domain()->SharesStorage(Domain().domain()));
    EXPECT_EQ(source.attachments().model().rows().count,11165u);
    const auto roles=runtime::ResolveSourceRoles(source);
    const auto& ledger=Model().coefficients();
    const auto bytes=runtime::detail::PackingBytes(Domain().domain().node_count(),128u<<20);
    auto packed=runtime::detail::PackOwner(ledger,Model().rigid_assembly(),roles,{15.6464,0,0},bytes);
    std::size_t endpoints=0,rigid=0,secondary=0,master=0;
    for (std::size_t n=0;n<roles.node.size();++n) {
        const bool beam=roles.node[n]&runtime::Beam18Endpoint;
        EXPECT_EQ(beam,ledger.nodes()[n].occurrences.beam18!=0);
        Same(packed.mass[n],ledger.nodes()[n].coefficients.mass);
        Same(packed.inertia[n],ledger.nodes()[n].coefficients.isotropic_inertia);
        if (!beam) continue;
        ++endpoints; rigid+=Model().rigid_assembly().FindMember(n)!=nullptr;
        secondary+=(roles.node[n]&runtime::CinSecondary)!=0; master+=(roles.node[n]&runtime::CinMaster)!=0;
        EXPECT_EQ(packed.rotation_present[n],1u);
    }
    EXPECT_EQ(endpoints,146u);
    auto bad=roles; bad.node.back()^=runtime::Beam18Endpoint;
    const auto first=packed.position.front();
    EXPECT_THROW(packed=runtime::detail::PackOwner(ledger,Model().rigid_assembly(),bad,{15.6464,0,0},bytes),std::runtime_error);
    Same(packed.position.front(),first);
    RecordProperty("beam_endpoint_nodes",endpoints); RecordProperty("beam_rigid_nodes",rigid);
    RecordProperty("beam_cin_secondary_nodes",secondary); RecordProperty("beam_cin_master_nodes",master);
    RecordProperty("attachment_forecast",std::to_string(source.forecast().total_bytes));
    RecordProperty("owner_packing_bytes",bytes);
}
} // namespace crash::cases::vehicle_startup::physical_model::supports_test
