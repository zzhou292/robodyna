#include <algorithm>
#include "../Internal.h"
#include "../../tests/Fixture.h"
#include "lib_utest/qualification/qbat_binding/Fixture.h"
#include "lib_src/assembly/ShellNodeMap.h"
#include "lib_src/assembly/NodalCoefficientLedger.h"
namespace crash::cases::vehicle_wall::native::physical_test {
namespace fe = tl::fea;
namespace d = physical_detail;
TEST(EnvelopeShellPacking, NewQephSuffixPreservesAllOriginalFamiliesAndNativeCoefficients) {
    qbat_binding_test::Fixture fixture;
    fe::ShellBatchBinding before,after;
    ASSERT_EQ(before.InitializeFormulations(fixture.Input()).status,fe::ShellBindingStatus::Success);
    const auto wall=detail::BuildGeometry(test::Box(),test::Decl(),test::Ids(),test::Units());
    d::shell::Inputs packed;
    packed.qeph.assign(fixture.q.begin(),fixture.q.end());packed.qeph.reserve(packed.qeph.size()+1);
    packed.t3.push_back(fixture.t);packed.qbat.push_back(fixture.b);
    packed.qeph.push_back(d::EnvironmentInput(wall.reference_input,test::Ids().shell,before.node_count()));
    ASSERT_EQ(after.InitializeFormulations(packed.Borrow(before.node_count()+4)).status,fe::ShellBindingStatus::Success);
    ASSERT_EQ(after.qeph_count(),before.qeph_count()+1);
    ASSERT_EQ(after.t3_count(),before.t3_count());
    ASSERT_EQ(after.qbat_count(),before.qbat_count());
    for(std::size_t i=0;i<before.qeph_count();++i) {
        EXPECT_EQ(after.qeph_source_id(i),before.qeph_source_id(i));
        EXPECT_EQ(after.qeph_nodes(i),before.qeph_nodes(i));
    }
    EXPECT_EQ(after.t3_nodes(0),before.t3_nodes(0));
    EXPECT_EQ(after.qbat_nodes(0),before.qbat_nodes(0));
    for(std::size_t i=0;i<before.node_count();++i) {
        EXPECT_EQ(after.active_nodes()[i].source_id,before.active_nodes()[i].source_id);
        qbat_binding_test::Exact(after.active_nodes()[i].native,before.active_nodes()[i].native);
    }
    std::vector<fe::NodalDomainNode> nodes;
    // Reverse the complete physical domain to prove that shell-local indices
    // are mapped by actual IDs, not copied as common-domain ordinals.
    for(const auto& node:after.active_nodes())nodes.push_back({node.source_id,node.position});
    std::reverse(nodes.begin(),nodes.end());
    fe::NodalNodeDomain domain;
    ASSERT_TRUE(domain.Initialize({41,nodes.data(),nodes.size()}));
    fe::ShellNodeMap map;
    ASSERT_TRUE(map.Initialize(after,domain));
    EXPECT_FALSE(map.identity_map());
    fe::NodalCoefficientLedger ledger;
    ASSERT_TRUE(ledger.Initialize({&map,nullptr,nullptr}));
    EXPECT_EQ(ledger.scope().uncovered_nodes,0u);
    for(unsigned k=0;k<4;++k) {
        const auto physical=domain.Find(test::Ids().nodes[k]);
        ASSERT_NE(physical,SIZE_MAX);
        EXPECT_EQ(output::Bits(ledger.nodes()[physical].coefficients.mass),output::Bits(wall.reference.nodal_mass[k]));
        EXPECT_EQ(output::Bits(ledger.nodes()[physical].coefficients.isotropic_inertia),
            output::Bits(wall.reference.isotropic_inertia[k]));
    }
}
TEST(EnvelopeShellPacking, InvalidSuffixIdentityAndCountNeverProduceAnInput) {
    const auto wall=detail::BuildGeometry(test::Box(),test::Decl(),test::Ids(),test::Units());
    EXPECT_THROW(d::EnvironmentInput(wall.reference_input,0,5),std::exception);
    EXPECT_THROW(d::EnvironmentInput(wall.reference_input,test::Ids().shell,SIZE_MAX),std::exception);
    const auto valid=d::EnvironmentInput(wall.reference_input,test::Ids().shell,5);
    EXPECT_EQ(valid.nodes,(std::array<std::size_t,4>{5,6,7,8}));
    EXPECT_EQ(valid.reference.node_ids[0],test::Ids().nodes[0]);
}
}
