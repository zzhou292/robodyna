#include "DirectFixture.h"
#include "../Storage.h"
#include "../SourceIds.h"
#include <gtest/gtest.h>
namespace crash::modelio::solid_control::test {
TEST(SolidControlDeclarations, DirectMembershipDoesNotRequireSelfContactOrResolveSharedMaterialClones) {
    Fixture fixture;
    const auto direct=fixture.Read();
    ASSERT_EQ(direct.parts.size(),1u);
    EXPECT_EQ(direct.parts[0].part_id,91u);
    EXPECT_EQ(direct.parts[0].original_solids,1u);
    // No SINGLE_SURFACE card exists; SOFT1 remains the legacy contact caller's preflight.
    EXPECT_EQ(direct.original_solids,1u);
    try {
        (void)detail::ResolveSharing({{19,401,702,"first",1},{91,401,701,"second",2}},
            {{401,"*SECTION_SOLID"}},{91});
        FAIL()<<"Expected later effective-property mapping rejection";
    } catch(const detail::Failure& error) { EXPECT_EQ(error.report.status,Status::NeedsNativePropertyMapping); }
}
TEST(SolidControlDeclarations, ParsedEvidenceOutlivesBorrowedMembersAndLateFailuresPreservePriorValues) {
    DirectData accepted;
    {
        Fixture fixture;accepted=fixture.Read();
        fixture.storage.clear();fixture.storage.shrink_to_fit();
    }
    ASSERT_FALSE(accepted.evidence.empty());
    EXPECT_EQ(accepted.evidence.front().block.keyword,"*CONTACT_INTERIOR");
    EXPECT_NE(accepted.evidence.front().block.raw_text.find("*CONTACT_INTERIOR"),std::string::npos);
    const auto before=accepted.combine_sha256;
    Fixture bad;bad.storage.back().second.back()='x';
    EXPECT_THROW(accepted=bad.Read(),std::exception);
    EXPECT_EQ(accepted.combine_sha256,before);
    Fixture nested(true),option(false,true);
    EXPECT_THROW(nested.Read(),std::exception);
    EXPECT_THROW(option.Read(),std::exception);
}
TEST(SolidControlDeclarations, SharedSectionExpandsOnlyEffectiveFlagAndPreservesUnavailableDisabledIdentity) {
    const auto ready=detail::ResolveSharing({{91,401,701,"first",9},{19,401,701,"second",3},
        {44,802,900,"other",1}},{{401,"*SECTION_SOLID"},{802,"*SECTION_SOLID"}},{91});
    ASSERT_EQ(ready.size(),3u);
    EXPECT_EQ(ready[0].part_id,19u);EXPECT_TRUE(ready[0].effective_control);
    EXPECT_FALSE(ready[0].directly_requested);EXPECT_EQ(ready[0].native_property_id,401u);
    EXPECT_FALSE(ready[1].effective_control);EXPECT_TRUE(ready[2].directly_requested);
    const auto disabled=detail::ResolveSharing({{91,401,701,"first",9},{19,401,702,"second",3}},
        {{401,"*SECTION_SOLID"}},{});
    for(const auto& row:disabled){EXPECT_FALSE(row.effective_control);EXPECT_EQ(row.native_property_id,0u);}
}
TEST(SolidControlDeclarations, RetainedAccountingIncludesStringCapacityAndOwnedEvidence) {
    Fixture fixture;auto values=fixture.Read();
    const auto before=detail::DirectBytes(values,sizeof(values));
    values.evidence[0].block.raw_text.reserve(values.evidence[0].block.raw_text.capacity()+4096);
    EXPECT_GT(detail::DirectBytes(values,sizeof(values)),before+4000);
    EffectiveData effective;effective.parts={{91,401,701,401,true,true}};
    effective.origins={{91,91,401,701,0,"source.key",std::string(64,'a'),1,4,3}};
    const auto initial=detail::EffectiveBytes(effective,sizeof(effective));
    effective.origins[0].member.reserve(2048);
    EXPECT_GT(detail::EffectiveBytes(effective,sizeof(effective)),initial+1900);
    std::size_t overflow=SIZE_MAX;
    EXPECT_THROW(detail::Charge(overflow,1),std::exception);
}
} // namespace crash::modelio::solid_control::test
