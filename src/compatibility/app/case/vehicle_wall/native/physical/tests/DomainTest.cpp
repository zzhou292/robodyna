#include <algorithm>
#include "modelio/physical_scope/DomainEmbedding.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <vector>
namespace crash::cases::vehicle_wall::native::physical_test {
namespace fe = tl::fea;
namespace domain = modelio::physical_scope;
namespace {
fe::NodalNodeDomain Make(const std::vector<fe::NodalDomainNode>& nodes, std::uint64_t instance=41) {
    fe::NodalNodeDomain result;
    const auto report=result.Initialize({instance,nodes.data(),nodes.size()});
    output::Require(bool(report),report.message);
    return result;
}
}
TEST(EnvelopeDomainValues, ExactUnsortedOriginalPrefixAndNewSuffixRetainEveryBit) {
    const std::vector<fe::NodalDomainNode> original{{7,{-0.,1,2}},{2,{3,4,5}}};
    const std::vector<fe::NodalDomainNode> suffix{{101,{7,8,9}},{102,{7,9,9}}};
    auto nodes=original;nodes.insert(nodes.end(),suffix.begin(),suffix.end());
    const auto before=Make(original),after=Make(nodes);
    const std::vector<std::uint64_t> canonical{2,7,99};
    EXPECT_NO_THROW(domain::detail::CheckEmbedding(canonical,before,after,{suffix.data(),suffix.size()}));
    EXPECT_EQ(before.node_count(),2u);
    EXPECT_FALSE(before.SharesStorage(after));
    EXPECT_EQ(output::Bits(before.nodes()[0].position.x),output::Bits(-0.));
}
TEST(EnvelopeDomainValues, ChangedOrderPositionSignedZeroAndOmittedCanonicalCollisionReject) {
    const std::vector<fe::NodalDomainNode> original{{7,{-0.,1,2}},{2,{3,4,5}}};
    const std::vector<fe::NodalDomainNode> suffix{{101,{7,8,9}}};
    const auto before=Make(original);
    const std::vector<std::uint64_t> canonical{2,7,99};
    for(unsigned variant=0;variant<5;++variant) {
        auto nodes=original;nodes.insert(nodes.end(),suffix.begin(),suffix.end());
        if(variant==0)std::swap(nodes[0],nodes[1]);
        if(variant==1)nodes[0].position.x=0.;
        if(variant==2)nodes[1].position.z=6.;
        if(variant==3)nodes.back().source_id=99;
        const auto after=Make(nodes,variant==4?42:41);
        const auto* declared=variant==3?nodes.data()+2:suffix.data();
        EXPECT_THROW(domain::detail::CheckEmbedding(canonical,before,after,{declared,1}),std::exception);
    }
    EXPECT_EQ(before.nodes()[1].source_id,2u);
}
TEST(EnvelopeDomainValues, MalformedSuffixSpanRejectsBeforeReadingAndExactRetrySucceeds) {
    const std::vector<fe::NodalDomainNode> original{{7,{0,1,2}}},suffix{{101,{7,8,9}}};
    auto nodes=original;nodes.push_back(suffix[0]);
    const auto before=Make(original),after=Make(nodes);
    const std::vector<std::uint64_t> canonical{7,99};
    EXPECT_THROW(domain::detail::CheckEmbedding(canonical,before,after,{nullptr,1}),std::exception);
    const auto* bad=reinterpret_cast<const fe::NodalDomainNode*>(std::uintptr_t{1});
    EXPECT_THROW(domain::detail::CheckEmbedding(canonical,before,after,{bad,1}),std::exception);
    EXPECT_THROW(domain::detail::CheckEmbedding(canonical,before,after,{suffix.data(),0}),std::exception);
    EXPECT_NO_THROW(domain::detail::CheckEmbedding(canonical,before,after,{suffix.data(),1}));
}
}
