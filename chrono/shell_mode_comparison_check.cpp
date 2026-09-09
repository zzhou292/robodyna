#include "ShellModeComparison.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace crash::reference {
namespace {
using Status=ShellModeComparisonStatus;

ShellTranslationMass UnitMass() {
    ShellTranslationMass mass;
    mass.fill(1);
    return mass;
}
void FreezeClusters(ShellModeSet& modes) {
    std::string diagnostic;
    if (ClusterShellFrequencies(modes.frequency,modes.mode_count,modes.cluster,diagnostic)!=Status::kSuccess)
        throw std::runtime_error(diagnostic);
}
ShellModeSet Single(double x=1,double y=0,double frequency=100) {
    ShellModeSet modes;
    modes.coordinate_count=2;
    modes.mode_count=1;
    modes.frequency[0]=frequency;
    modes.translation[0][0]=x;
    modes.translation[0][1]=y;
    return modes;
}
ShellModeSet Plane() {
    auto modes=Single();
    modes.coordinate_count=3;
    modes.mode_count=2;
    modes.frequency[1]=100.5;
    modes.translation[1][1]=1;
    FreezeClusters(modes);
    return modes;
}
void ExpectFailure(const ShellModeSet& reference,const ShellModeSet& candidate,
                   const ShellTranslationMass& mass,Status expected) {
    ShellModeComparison output;
    output.mode_count=17;
    output.cluster_count=19;
    output.minimum_squared_cosine=.25;
    output.maximum_relative_frequency_change=.75;
    output.singleton_candidate.fill(13);
    for (auto& group:output.clusters) {
        group.reference_cluster=7;
        group.candidate_cluster=11;
        group.mode_count=3;
        group.reference_modes.fill(4);
        group.candidate_modes.fill(5);
        group.minimum_squared_cosine=.125;
        group.maximum_relative_frequency_change=.375;
    }
    std::array<unsigned char,sizeof(output)> before;
    std::memcpy(before.data(),&output,sizeof(output));
    std::string diagnostic;
    EXPECT_EQ(CompareShellModes(reference,candidate,mass,output,diagnostic),expected) << diagnostic;
    EXPECT_EQ(std::memcmp(before.data(),&output,sizeof(output)),0);
    EXPECT_FALSE(diagnostic.empty());
}

TEST(ShellModeComparisonCheck, ConnectedClustersUseFrozenLowerFrequencyGapAndStageFailures) {
    ShellModeSet modes;
    modes.mode_count=4;
    modes.frequency[0]=101.8;
    modes.frequency[1]=100;
    modes.frequency[2]=100.9;
    modes.frequency[3]=110;
    FreezeClusters(modes);
    EXPECT_EQ(modes.cluster[0],0);
    EXPECT_EQ(modes.cluster[1],0);
    EXPECT_EQ(modes.cluster[2],0);
    EXPECT_EQ(modes.cluster[3],1);
    // The connected cluster spans 1.8% even though every adjacent gap is <1%.
    modes.mode_count=2;
    modes.frequency[0]=100;
    modes.frequency[1]=101;
    FreezeClusters(modes);
    EXPECT_EQ(modes.cluster[0],modes.cluster[1]);
    modes.frequency[1]=std::nextafter(101.,102.);
    FreezeClusters(modes);
    EXPECT_NE(modes.cluster[0],modes.cluster[1]);
    auto output=modes.cluster;
    const auto before=output;
    std::string diagnostic;
    modes.frequency[1]=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(ClusterShellFrequencies(modes.frequency,2,output,diagnostic),Status::kInvalidInput);
    EXPECT_EQ(output,before);
    EXPECT_EQ(ClusterShellFrequencies(modes.frequency,0,output,diagnostic),Status::kInvalidInput);
    EXPECT_EQ(output,before);
    EXPECT_EQ(ClusterShellFrequencies(modes.frequency,25,output,diagnostic),Status::kInvalidInput);
    EXPECT_EQ(output,before);
}

TEST(ShellModeComparisonCheck, SingletonSignScaleAndInputPermutationPreserveMatches) {
    ShellModeSet reference;
    reference.coordinate_count=3;
    reference.mode_count=3;
    for (std::size_t i=0;i<3;++i) {
        reference.frequency[i]=10*(i+1);
        reference.translation[i][i]=1;
    }
    FreezeClusters(reference);
    auto candidate=reference;
    const std::array<std::size_t,3> permutation{2,0,1};
    for (std::size_t i=0;i<3;++i) {
        candidate.frequency[i]=reference.frequency[permutation[i]];
        candidate.translation[i]={};
        candidate.translation[i][permutation[i]]=i==1 ? -1e250 : 1e-250;
    }
    FreezeClusters(candidate);
    ShellModeComparison output;
    std::string diagnostic;
    ASSERT_EQ(CompareShellModes(reference,candidate,UnitMass(),output,diagnostic),Status::kSuccess) << diagnostic;
    EXPECT_EQ(output.singleton_candidate[0],1);
    EXPECT_EQ(output.singleton_candidate[1],2);
    EXPECT_EQ(output.singleton_candidate[2],0);
    EXPECT_NEAR(output.minimum_squared_cosine,1,1e-14);
    EXPECT_DOUBLE_EQ(output.maximum_relative_frequency_change,0);
}

TEST(ShellModeComparisonCheck, CommonPhysicalMassMatchesIndependentTwoCoordinateOracle) {
    const auto reference=Single(1,1);
    const auto candidate=Single(1,.9);
    auto mass=UnitMass();
    mass[1]=4;
    // Direct weighted dot products, independent of the production SVD path.
    const long double dot=1.L+4.L*.9L;
    const long double a_squared=5.L,b_squared=1.L+4.L*.9L*.9L;
    const double oracle=static_cast<double>(dot*dot/(a_squared*b_squared));
    ShellModeComparison output;
    std::string diagnostic;
    ASSERT_EQ(CompareShellModes(reference,candidate,mass,output,diagnostic),Status::kSuccess) << diagnostic;
    EXPECT_NEAR(output.minimum_squared_cosine,oracle,2e-14);
    const auto tilted=Single(1,.1);
    ASSERT_EQ(CompareShellModes(Single(),tilted,UnitMass(),output,diagnostic),Status::kSuccess) << diagnostic;
    EXPECT_NEAR(output.minimum_squared_cosine,1/1.01,2e-14);
    ExpectFailure(Single(),tilted,mass,Status::kUnmatched);
}

TEST(ShellModeComparisonCheck, RotatedClusterBasisAndLabelsPreserveSubspaceWithoutVectorPairing) {
    auto reference=Plane(),candidate=reference;
    // In the common metric diag(1,4,1), these span the same plane with a 45deg
    // basis rotation. Neither individual candidate vector has MAC >=.99.
    candidate.translation[0]={1,.5,0};
    candidate.translation[1]={-1,.5,0};
    candidate.frequency[0]=100.5;
    candidate.frequency[1]=100;
    candidate.cluster[0]=candidate.cluster[1]=17;
    auto mass=UnitMass();
    mass[1]=4;
    ShellModeComparison output;
    std::string diagnostic;
    ASSERT_EQ(CompareShellModes(reference,candidate,mass,output,diagnostic),Status::kSuccess) << diagnostic;
    EXPECT_EQ(output.cluster_count,1);
    EXPECT_EQ(output.clusters[0].candidate_cluster,17);
    EXPECT_EQ(output.clusters[0].mode_count,2);
    EXPECT_NEAR(output.minimum_squared_cosine,1,2e-14);
    EXPECT_EQ(output.singleton_candidate[0],kNoShellMode);
    EXPECT_EQ(output.singleton_candidate[1],kNoShellMode);
    EXPECT_DOUBLE_EQ(output.maximum_relative_frequency_change,0);
    candidate.frequency[0]=104.5;
    candidate.frequency[1]=104;
    ASSERT_EQ(CompareShellModes(reference,candidate,mass,output,diagnostic),Status::kSuccess) << diagnostic;
    EXPECT_NEAR(output.maximum_relative_frequency_change,.04,1e-16);
    candidate.frequency[0]=106.5;
    candidate.frequency[1]=106;
    ExpectFailure(reference,candidate,mass,Status::kFrequencyMismatch);
}

TEST(ShellModeComparisonCheck, PrincipalAnglesUseTheWorstSubspaceDirection) {
    const auto reference=Plane();
    auto candidate=reference;
    candidate.translation[1]={0,std::sqrt(.995),std::sqrt(.005)};
    ShellModeComparison output;
    std::string diagnostic;
    ASSERT_EQ(CompareShellModes(reference,candidate,UnitMass(),output,diagnostic),Status::kSuccess) << diagnostic;
    EXPECT_NEAR(output.minimum_squared_cosine,.995,2e-14);
    candidate.translation[1]={0,std::sqrt(.98),std::sqrt(.02)};
    ExpectFailure(reference,candidate,UnitMass(),Status::kUnmatched);
    candidate.translation[1]={0,0,1};
    ExpectFailure(reference,candidate,UnitMass(),Status::kUnmatched);
    ExpectFailure(Single(),Single(0,1),UnitMass(),Status::kUnmatched);
}

TEST(ShellModeComparisonCheck, RankLossIsRejectedWithoutDroppingProjectedDirections) {
    const auto reference=Plane();
    auto candidate=reference;
    candidate.translation[1]=candidate.translation[0];
    ExpectFailure(reference,candidate,UnitMass(),Status::kRankDeficient);
    candidate.translation[1][1]=1e-10;
    ExpectFailure(reference,candidate,UnitMass(),Status::kRankDeficient);
    ExpectFailure(Single(),Single(0,0),UnitMass(),Status::kRankDeficient);
    auto overfull=reference;
    overfull.coordinate_count=2;
    overfull.mode_count=3;
    overfull.frequency[2]=100.6;
    overfull.translation[2][0]=1;
    FreezeClusters(overfull);
    auto selected=Single();
    ExpectFailure(selected,overfull,UnitMass(),Status::kRankDeficient);
    auto extreme=Single(1,std::numeric_limits<double>::denorm_min());
    auto mass=UnitMass();
    mass[0]=std::numeric_limits<double>::max();
    mass[1]=std::numeric_limits<double>::denorm_min();
    ExpectFailure(Single(),extreme,mass,Status::kNumericalFailure);
}

TEST(ShellModeComparisonCheck, AmbiguousShapeIdentityCannotBeBrokenByFrequencyOrInputOrder) {
    const auto reference=Single();
    auto candidate=reference;
    candidate.mode_count=2;
    candidate.frequency[1]=200;
    candidate.translation[1][0]=-2;
    FreezeClusters(candidate);
    ExpectFailure(reference,candidate,UnitMass(),Status::kAmbiguous);
    std::swap(candidate.frequency[0],candidate.frequency[1]);
    FreezeClusters(candidate);
    ExpectFailure(reference,candidate,UnitMass(),Status::kAmbiguous);
    // Two selected reference branches may not consume the same candidate.
    auto duplicate_reference=candidate;
    ExpectFailure(duplicate_reference,reference,UnitMass(),Status::kAmbiguous);
}

TEST(ShellModeComparisonCheck, ExtraCandidateClustersAndFrequencyOrderCrossingsAreAllowed) {
    auto reference=Single();
    reference.mode_count=2;
    reference.frequency[1]=102;
    reference.translation[1][1]=1;
    FreezeClusters(reference);
    auto candidate=reference;
    candidate.frequency[0]=104;
    candidate.frequency[1]=101;
    FreezeClusters(candidate);
    ShellModeComparison output;
    std::string diagnostic;
    ASSERT_EQ(CompareShellModes(reference,candidate,UnitMass(),output,diagnostic),Status::kSuccess) << diagnostic;
    EXPECT_EQ(output.singleton_candidate[0],0);
    EXPECT_EQ(output.singleton_candidate[1],1);
    EXPECT_NEAR(output.maximum_relative_frequency_change,.04,1e-16);
    // The selected first mode has one geometric match in a larger pool.
    reference.mode_count=1;
    FreezeClusters(reference);
    ASSERT_EQ(CompareShellModes(reference,candidate,UnitMass(),output,diagnostic),Status::kSuccess) << diagnostic;
    EXPECT_EQ(output.mode_count,1);
    EXPECT_EQ(output.cluster_count,1);
    EXPECT_EQ(output.singleton_candidate[0],0);
}

TEST(ShellModeComparisonCheck, MatchedFrequencyThresholdIsInclusiveAndReferenceRelative) {
    const auto reference=Single();
    ShellModeComparison output;
    std::string diagnostic;
    for (const double edge:{95.,105.}) {
        SCOPED_TRACE(edge);
        auto candidate=Single(1,0,edge);
        ASSERT_EQ(CompareShellModes(reference,candidate,UnitMass(),output,diagnostic),Status::kSuccess) << diagnostic;
        EXPECT_DOUBLE_EQ(output.maximum_relative_frequency_change,.05);
        candidate.frequency[0]=std::nextafter(edge,edge<100 ? 0. : 200.);
        ExpectFailure(reference,candidate,UnitMass(),Status::kFrequencyMismatch);
    }
    // A late failure after a valid first cluster must preserve all outputs.
    auto a=reference,b=reference;
    a.mode_count=b.mode_count=2;
    a.frequency[1]=200;
    b.frequency[1]=211;
    a.translation[1][1]=b.translation[1][1]=1;
    FreezeClusters(a);
    FreezeClusters(b);
    ExpectFailure(a,b,UnitMass(),Status::kFrequencyMismatch);
}

TEST(ShellModeComparisonCheck, MalformedDimensionsMassValuesAndClusterPartitionsPreserveOutput) {
    const auto reference=Single();
    auto candidate=reference;
    candidate.coordinate_count=3;
    ExpectFailure(reference,candidate,UnitMass(),Status::kDimensionMismatch);
    candidate=reference;
    candidate.mode_count=25;
    ExpectFailure(reference,candidate,UnitMass(),Status::kInvalidInput);
    candidate=reference;
    candidate.coordinate_count=13;
    ExpectFailure(reference,candidate,UnitMass(),Status::kInvalidInput);
    for (const double invalid:{0.,-1.,std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::quiet_NaN()}) {
        SCOPED_TRACE(invalid);
        auto mass=UnitMass();
        mass[1]=invalid;
        ExpectFailure(reference,reference,mass,Status::kInvalidInput);
        candidate=reference;
        candidate.frequency[0]=invalid;
        ExpectFailure(reference,candidate,UnitMass(),Status::kInvalidInput);
    }
    candidate=reference;
    candidate.translation[0][1]=std::numeric_limits<double>::quiet_NaN();
    ExpectFailure(reference,candidate,UnitMass(),Status::kInvalidInput);
    candidate=reference;
    candidate.cluster[0]=24;
    ExpectFailure(reference,candidate,UnitMass(),Status::kInvalidInput);
    auto plane=Plane();
    candidate=plane;
    candidate.cluster[1]=1;
    ExpectFailure(plane,candidate,UnitMass(),Status::kInvalidInput);
    auto singleton=reference;
    singleton.coordinate_count=3;
    ExpectFailure(singleton,plane,UnitMass(),Status::kDimensionMismatch);
}

TEST(ShellModeComparisonCheck, FullBoundedCapacitySupportsDistinctNonorthogonalSingletons) {
    ShellModeSet modes;
    modes.coordinate_count=kShellTranslationCapacity;
    modes.mode_count=kShellModeCapacity;
    for (std::size_t i=0;i<kShellModeCapacity;++i) {
        modes.frequency[i]=i+1;
        modes.translation[i][i%kShellTranslationCapacity]=1;
        if (i>=kShellTranslationCapacity)
            modes.translation[i][(i+1)%kShellTranslationCapacity]=1;
    }
    FreezeClusters(modes);
    ShellModeComparison output;
    std::string diagnostic;
    ASSERT_EQ(CompareShellModes(modes,modes,UnitMass(),output,diagnostic),Status::kSuccess) << diagnostic;
    EXPECT_EQ(output.mode_count,24);
    EXPECT_EQ(output.cluster_count,24);
    for (std::size_t i=0;i<kShellModeCapacity;++i) EXPECT_EQ(output.singleton_candidate[i],i);
    EXPECT_NEAR(output.minimum_squared_cosine,1,2e-14);
}
}  // namespace
}  // namespace crash::reference
