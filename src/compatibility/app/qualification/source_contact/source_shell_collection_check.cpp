#include "SourceShellCollection.h"
#include "SourceShellReferenceInput.h"
#include "SourcePartT3ReferenceInput.h"
#include "lib_utest/qualification/native/t3/T3StartupTestOracle.h"
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {
namespace source = crash::qualification::source_contact;
namespace fe = tl::fea;
namespace oracle = tl::qualification::t3::test;
std::filesystem::path readiness;
struct Mass { long double mass=0, total=0, physical=0, added=0; };
void Add(Mass& sum, const Mass& term) {
    sum.mass += term.mass; sum.total += term.total;
    sum.physical += term.physical; sum.added += term.added;
}
void Check(const fe::ShellBindingMass& actual, const Mass& expected) {
    oracle::Near(actual.mass, expected.mass, expected.mass);
    oracle::Near(actual.isotropic_inertia, expected.total, expected.total);
    oracle::Near(actual.physical_inertia, expected.physical, expected.physical);
    oracle::Near(actual.added_inertia, expected.added, expected.added);
}
std::string Exact(double x) { std::ostringstream s; s << std::setprecision(17) << x; return s.str(); }
template<class Input>
void CheckMaterial(const Input& in, const source::SourcePartContactFixture& source) {
    EXPECT_EQ(in.density,source.surface_mass().density_kg_m3);
    EXPECT_EQ(in.thickness,source.surface_mass().thickness_m);
    EXPECT_EQ(in.young_modulus,200e9); EXPECT_EQ(in.poisson_ratio,.3);
}
class SourceShellCollectionCheck : public ::testing::Test {
  protected:
    source::SourcePartContactFixture fixture;
    void SetUp() override {
        const auto result = source::LoadPinnedSourcePartContact(readiness, &fixture);
        ASSERT_EQ(result.status, source::FixtureStatus::Ok) << result.diagnostic;
    }
};

TEST_F(SourceShellCollectionCheck, AllOriginalParentsBuildNativeStructuralMassOnAll117Nodes) {
    source::SourceShellCollection inputs;
    ASSERT_EQ(inputs.Initialize(fixture).status, source::FixtureStatus::Ok);
    fe::ShellBatchBinding binding;
    const auto result = binding.Initialize(inputs.input());
    ASSERT_EQ(result.status, fe::ShellBindingStatus::Success) << result.message;
    ASSERT_EQ(binding.node_count(), source::NodeCount);
    ASSERT_EQ(binding.qeph_count(), source::Q4Count);
    ASSERT_EQ(binding.t3_count(), source::T3Count);
    std::array<Mass, source::NodeCount> expected{};
    std::array<bool, source::ParentCount> visited{};
    for (std::size_t e=0; e<source::Q4Count; ++e) {
        const auto p=inputs.qeph_source_parents()[e]; ASSERT_LT(p, source::ParentCount);
        ASSERT_FALSE(visited[p]); visited[p]=true;
        EXPECT_EQ(binding.qeph_source_id(e), fixture.parents()[p].source_id);
        const auto& ref=binding.qeph_reference(e);
        CheckMaterial(ref.input,fixture);
        const auto& x=ref.input.position;
        // Independent world-diagonal projected area, valid for these original
        // warped native Q4s; display triangulation/proxy area is not substituted.
        const auto area=oracle::CrossNorm(oracle::Difference(x[2],x[0]),
                                           oracle::Difference(x[3],x[1]))/2;
        const long double mass=static_cast<long double>(ref.input.density)*ref.input.thickness*area/4;
        const long double physical=mass*ref.input.thickness*ref.input.thickness/12, added=mass*area/12;
        for (unsigned n=0; n<4; ++n) {
            EXPECT_EQ(binding.qeph_nodes(e)[n], fixture.parents()[p].local_node_indices[n]);
            EXPECT_EQ(ref.input.node_ids[n], fixture.parents()[p].raw_record[2+n]);
            Add(expected[binding.qeph_nodes(e)[n]], {mass,physical+added,physical,added});
        }
    }
    for (std::size_t e=0; e<source::T3Count; ++e) {
        const auto p=inputs.t3_source_parents()[e]; ASSERT_LT(p, source::ParentCount);
        ASSERT_FALSE(visited[p]); visited[p]=true;
        EXPECT_EQ(binding.t3_source_id(e), fixture.parents()[p].source_id);
        CheckMaterial(binding.t3_reference(e).input,fixture);
        const auto truth=oracle::Independent(source::T3ReferenceInput(fixture,p));
        for (unsigned n=0; n<3; ++n) {
            EXPECT_EQ(binding.t3_nodes(e)[n], fixture.parents()[p].local_node_indices[n]);
            EXPECT_EQ(binding.t3_reference(e).input.node_ids[n], fixture.parents()[p].raw_record[2+n]);
            const auto w=truth.weight[n];
            Add(expected[binding.t3_nodes(e)[n]], {truth.mass*w,truth.total*w,truth.physical*w,truth.added*w});
        }
    }
    for (bool found : visited) EXPECT_TRUE(found);
    Mass total;
    for (std::size_t n=0; n<source::NodeCount; ++n) {
        SCOPED_TRACE(n);
        const auto& node=binding.nodes()[n];
        EXPECT_EQ(node.source_id, fixture.nodes()[n].source_id);
        const double xyz[]{node.position.x,node.position.y,node.position.z};
        EXPECT_EQ(std::memcmp(xyz, fixture.coordinates().data()+3*n, sizeof(xyz)),0);
        ASSERT_GT(node.native.mass,0); ASSERT_GT(node.native.isotropic_inertia,0);
        Check(node.native,expected[n]); Add(total,expected[n]);
    }
    Check(binding.totals(),total);
    RecordProperty("source_readiness_sha256",source::ReadinessSha256);
    RecordProperty("original_nodes",117); RecordProperty("original_q4",88); RecordProperty("original_t3",6);
    RecordProperty("native_structural_mass_kg",Exact(binding.totals().mass));
    RecordProperty("native_total_inertia_kg_m2",Exact(binding.totals().isotropic_inertia));
    RecordProperty("native_physical_inertia_kg_m2",Exact(binding.totals().physical_inertia));
    RecordProperty("native_added_inertia_kg_m2",Exact(binding.totals().added_inertia));
    RecordProperty("scope","Original geometry and source rho/t, explicit elastic LAW1; no MAT024/attachments/dynamics admission");
}

TEST_F(SourceShellCollectionCheck, ConversionRejectsUnpreparedOrWrongFamilyAndKeepsImmutableSourceMap) {
    source::SourceShellCollection inputs;
    source::SourcePartContactFixture empty;
    EXPECT_EQ(inputs.Initialize(empty).status,source::FixtureStatus::InvalidFixture);
    EXPECT_FALSE(inputs.prepared()); EXPECT_EQ(inputs.input().node_count,0);
    ASSERT_EQ(inputs.Initialize(fixture).status,source::FixtureStatus::Ok);
    const auto before=inputs.input();
    const auto qmap=inputs.qeph_source_parents();
    const auto tmap=inputs.t3_source_parents();
    EXPECT_THROW(source::QephReferenceInput(fixture,tmap[0]),std::invalid_argument);
    EXPECT_THROW(source::T3PortReferenceInput(fixture,qmap[0]),std::invalid_argument);
    EXPECT_THROW(source::QephReferenceInput(fixture,source::ParentCount),std::invalid_argument);
    EXPECT_THROW(source::T3PortReferenceInput(empty,0),std::invalid_argument);
    EXPECT_EQ(inputs.Initialize(empty).status,source::FixtureStatus::InvalidArgument);
    EXPECT_EQ(inputs.Initialize(fixture).status,source::FixtureStatus::InvalidArgument);
    EXPECT_EQ(inputs.input().qeph,before.qeph); EXPECT_EQ(inputs.input().t3,before.t3);
    EXPECT_EQ(inputs.qeph_source_parents(),qmap); EXPECT_EQ(inputs.t3_source_parents(),tmap);
    fe::ShellBatchBinding actual, clean;
    source::SourceShellCollection rebuilt; ASSERT_EQ(rebuilt.Initialize(fixture).status,source::FixtureStatus::Ok);
    ASSERT_EQ(actual.Initialize(inputs.input()).status,fe::ShellBindingStatus::Success);
    ASSERT_EQ(clean.Initialize(rebuilt.input()).status,fe::ShellBindingStatus::Success);
    EXPECT_EQ(actual.inventory(),clean.inventory());
}
}  // namespace
int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if (argc!=2) { std::cerr << "Usage: source_shell_collection_check READINESS.json\n"; return 2; }
    readiness=argv[1]; return RUN_ALL_TESTS();
}
