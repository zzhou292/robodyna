#include "case/vehicle_runtime/SourceRoles.h"
#include "lib_utest/qualification/nodal_coefficients/SolidFixture.h"
#include "lib_src/constraints/tied_shell/TiedCinAttachmentModel.h"
#include "lib_src/constraints/NodalRigidAssemblyBinding.h"
namespace crash::cases::vehicle_wall::native::execution_test {
namespace fe=tl::fea;
namespace rt=vehicle_runtime;
namespace tied=tl::constraints::tied_shell;
struct RoleFixture {
    coefficient_test::SolidFixture source;
    fe::NodalNodeDomain domain;
    fe::ShellNodeMap shells;
    fe::type13::Model beams;
    fe::Type13NodeContributions beam_values;
    fe::type25::Model welds;
    fe::SolidNodeContributions solids;
    fe::ElementMassContributions mass;
    fe::NodalCoefficientLedger ledger;
    fe::NodalRigidAssemblyBinding rigid;
    tied::TiedCinAttachmentModel cin;
    RoleFixture():domain(source.Domain()),shells(source.Map(domain)),beams(source.Beams()),
        welds(source.Springs()),solids(source.Solids(domain)) {
        EXPECT_TRUE(beam_values.Initialize(beams,domain));
        const fe::ElementMassSource point{18000,777,domain.Find(777),.002};
        EXPECT_TRUE(mass.Initialize(domain,{1,1,&point,1}));
        EXPECT_TRUE(ledger.InitializeWithSolids({{&shells,&welds,&beam_values},&mass,&solids}));
        EXPECT_TRUE(rigid.InitializeEmpty(ledger));
        EXPECT_TRUE(tied::PrepareEmptyCinAttachments(domain,&cin));
    }
};
TEST(EnvelopeSourceRoles, GenuineHeterogeneousIncidenceRetainsOnlyActualEndpointAndShellRoles) {
    RoleFixture f;
    const auto roles=rt::ResolvePhysicalSourceRoles(f.ledger,f.rigid,f.cin,f.beams,nullptr);
    ASSERT_EQ(roles.node.size(),f.domain.node_count());
    for(std::size_t i=0;i<roles.node.size();++i) {
        const auto& c=f.ledger.nodes()[i].occurrences;
        EXPECT_EQ(bool(roles.node[i]&rt::Shell),bool(c.qeph||c.t3||c.qbat));
        EXPECT_EQ(bool(roles.node[i]&rt::Type13Endpoint),bool(c.type13));
        EXPECT_EQ(bool(roles.node[i]&rt::Type25Endpoint),bool(c.type25));
        EXPECT_EQ(roles.node[i]&(rt::Part|rt::PlainRigid|rt::CinSecondary|rt::CinMaster|rt::Beam18Endpoint),0);
    }
    EXPECT_EQ(roles.node[f.domain.Find(777)],0); // Real point mass supplies no rotational source role.
}
TEST(EnvelopeSourceRoles, ForeignConstraintDomainAndMissingActualBeamSourceReject) {
    RoleFixture f;
    const auto foreign=f.source.Domain();tied::TiedCinAttachmentModel cin;
    ASSERT_TRUE(tied::PrepareEmptyCinAttachments(foreign,&cin));
    EXPECT_THROW(rt::ResolvePhysicalSourceRoles(f.ledger,f.rigid,cin,f.beams,nullptr),std::exception);
    fe::type13::Model absent;
    EXPECT_THROW(rt::ResolvePhysicalSourceRoles(f.ledger,f.rigid,f.cin,absent,nullptr),std::exception);
    EXPECT_THROW(rt::ResolvePhysicalSourceRoles(f.ledger,f.rigid,f.cin,f.beams,nullptr,1),std::exception);
    EXPECT_NO_THROW(rt::ResolvePhysicalSourceRoles(f.ledger,f.rigid,f.cin,f.beams,nullptr));
}
}
