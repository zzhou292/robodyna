#include "Fixture.h"
#include "../ActivityValues.h"
namespace crash::cases::vehicle_startup::cin_witness_test {
TEST(TiedCinWitnessActivityValues, ActualFamilyIndexMappingKeepsCoincidentPositiveWitnessAndPendingDistinct) {
    Fixture f;
    const auto roster=f.Build();
    std::uint8_t q[]{0,1},t[]{0},b[]{0};
    cin_witness_detail::FamilyActivity input{q,t,b,2,1,1};
    std::vector<std::uint8_t> flags(roster.witnesses.size(),19);
    auto report=cin_witness_detail::MapActivity(roster,input,flags.data(),flags.size());
    EXPECT_EQ(report.status,TiedCinActivityStatus::Success);
    EXPECT_EQ(flags,(std::vector<std::uint8_t>{2,1,2,2,1,2,2}));
    // Declared masters are inactive, but the separate same-connectivity layer
    // remains a sufficient positive witness for both native patch topologies.
    q[1]=0;
    report=cin_witness_detail::MapActivity(roster,input,flags.data(),flags.size());
    EXPECT_EQ(report.status,TiedCinActivityStatus::PendingPositiveShellWitness);
    EXPECT_EQ(report.row,0u);
    EXPECT_EQ(flags,std::vector<std::uint8_t>(flags.size(),2));
    // An active T3 cannot keep the four-distinct-node quad patch eligible.
    t[0]=1;
    report=cin_witness_detail::MapActivity(roster,input,flags.data(),flags.size());
    EXPECT_EQ(report.status,TiedCinActivityStatus::PendingPositiveShellWitness);
    EXPECT_EQ(report.row,0u);
    EXPECT_EQ(flags[5],1);
}
TEST(TiedCinWitnessActivityValues, LateInvalidValueAssociationAndAliasesPreserveOutputThenRetry) {
    Fixture f;
    auto roster=f.Build();
    std::uint8_t q[]{1,1},t[]{1},b[]{1};
    cin_witness_detail::FamilyActivity input{q,t,b,2,1,1};
    std::vector<std::uint8_t> output(roster.witnesses.size(),19);
    const auto saved=output;
    q[0]=3;
    EXPECT_EQ(cin_witness_detail::MapActivity(roster,input,output.data(),output.size()).status,
              TiedCinActivityStatus::InvalidInput);
    EXPECT_EQ(output,saved);
    q[0]=1;
    ++roster.witnesses.back().native_parent_index;
    ++roster.witnesses.back().native_parent_index;
    EXPECT_EQ(cin_witness_detail::MapActivity(roster,input,output.data(),output.size()).status,
              TiedCinActivityStatus::InvalidInput);
    EXPECT_EQ(output,saved);
    roster=f.Build();
    auto alias=input;
    alias.qeph=output.data();
    EXPECT_EQ(cin_witness_detail::MapActivity(roster,alias,output.data(),output.size()).status,
              TiedCinActivityStatus::InvalidInput);
    EXPECT_EQ(output,saved);
    EXPECT_EQ(cin_witness_detail::MapActivity(roster,input,output.data(),output.size()).status,
              TiedCinActivityStatus::Success);
    EXPECT_EQ(output,std::vector<std::uint8_t>(output.size(),1));
}
TEST(TiedCinWitnessActivityValues, WorkspaceAndCompleteHostCapsAreExplicitBeforeAllocation) {
    const auto budget=cin_witness_detail::ActivityBudget(10000,349645,30000,1000,{});
    EXPECT_EQ(budget.workspace_bytes,410645u);
    TiedCinActivityLimits limits;
    limits.workspace_bytes=budget.workspace_bytes-1;
    EXPECT_THROW(cin_witness_detail::ActivityBudget(10000,349645,30000,1000,limits),std::exception);
    ++limits.workspace_bytes;
    EXPECT_EQ(cin_witness_detail::ActivityBudget(10000,349645,30000,1000,limits).workspace_bytes,limits.workspace_bytes);
    EXPECT_THROW(cin_witness_detail::ActivityBudget(TiedCinWitnessLimits{}.host_bytes,349645,30000,1000,{}),std::exception);
}
}
