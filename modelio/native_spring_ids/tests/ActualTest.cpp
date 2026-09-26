#include "ActualFixture.h"
#include "Observed.h"
#include <fstream>
#include <unistd.h>
namespace crash::modelio::native_spring_ids::test {
namespace {
std::filesystem::path ObservedDirectory() {
    const auto* value=std::getenv("ROBO_NATIVE_SPRING_OBSERVED");
    output::Require(value && *value,"Missing independent observed SPRING evidence directory"); return value;
}
void ReadySources() {
    ASSERT_EQ(ActualImportContext().data().diagnostic.status,Readiness::Ready)
        << ActualImportContext().data().diagnostic.reason;
    ASSERT_EQ(ActualResolution().diagnostic.status,Readiness::Ready) << ActualResolution().diagnostic.reason;
}
struct Temporary {
    std::filesystem::path path;
    Temporary() {
        char name[]="/tmp/robo-spring-observed-XXXXXX";const auto* value=mkdtemp(name);
        if(!value)throw std::runtime_error("Temporary evidence directory failed");path=value;
    }
    ~Temporary(){std::error_code error;std::filesystem::remove_all(path,error);}
};
}
TEST(NativeSpringIdsActual, ForecastCompleteSourceBeforeMappingAllocation) {
    ASSERT_EQ(ActualImportContext().data().diagnostic.status,Readiness::Ready)
        << ActualImportContext().data().diagnostic.reason;
    const auto forecast=Preflight(physical::Inputs().beams,physical::Model().welds(),
        physical::JointSource(),ActualImportContext());
    RecordProperty("complete_source_reservation",std::to_string(forecast.total_bytes));
    RecordProperty("integer_workspace_reservation",std::to_string(forecast.result_reservation));
    RecordProperty("context_reservation",std::to_string(ActualImportContext().data().forecast.total_bytes));
    EXPECT_LE(forecast.total_bytes,Limits{}.host_bytes);
    EXPECT_LE(forecast.result_reservation,Limits{}.resolve_bytes);
}
TEST(NativeSpringIdsActual, CompleteSourceNamespaceMatchesEveryObservedNativeIdAndEndpoint) {
    ASSERT_NO_FATAL_FAILURE(ReadySources());
    const auto observed=LoadObserved(ObservedDirectory());const auto& result=ActualResolution();
    ASSERT_EQ(result.rows.size(),observed.native_table.size()); ASSERT_EQ(result.rows.size(),7349u);
    EXPECT_EQ(result.counts.type13,4442u);EXPECT_EQ(result.counts.discrete_namespace_only,35u);
    EXPECT_EQ(result.counts.non_spring_beams,243u);EXPECT_EQ(result.counts.default_welds,2828u);
    EXPECT_EQ(result.counts.regular_joints,44u);EXPECT_EQ(result.physical_order.size(),7314u);
    for(std::size_t i=0;i<result.rows.size();++i) {
        SCOPED_TRACE(i);
        const auto& actual=result.rows[i];const auto& expected=observed.native_table[i];
        EXPECT_EQ(actual.native_id,expected.native_id);EXPECT_EQ(actual.endpoints[0],expected.node1);EXPECT_EQ(actual.endpoints[1],expected.node2);
        if(actual.kind==SourceKind::DiscreteNamespaceOnly) {
            EXPECT_FALSE(actual.physical_participant);EXPECT_EQ(actual.source_index,SIZE_MAX);EXPECT_EQ(actual.original_id,actual.native_id);
        }
    }
    for(const auto& expected:observed.generated) {
        SCOPED_TRACE(expected.source_id);
        const auto kind=expected.keyword=="*CONSTRAINED_SPOTWELD_ID"?SourceKind::DefaultSpotweld:SourceKind::RegularJoint;
        const auto* actual=result.Find(kind,expected.source_id);ASSERT_NE(actual,nullptr);
        EXPECT_EQ(actual->native_id,expected.native_id);EXPECT_EQ(actual->endpoints[0],expected.node1);EXPECT_EQ(actual->endpoints[1],expected.node2);
    }
    for(const auto index:result.physical_order) {
        ASSERT_LT(index,result.rows.size());const auto& row=result.rows[index];
        if(row.kind==SourceKind::Type13) {
            ASSERT_LT(row.source_index,physical::Inputs().beams.data().beams.size());
            const auto& source=physical::Inputs().beams.data().beams[row.source_index];
            EXPECT_EQ(source.id,row.original_id);EXPECT_EQ(source.canonical_index,row.canonical_index);
        } else if(row.kind==SourceKind::DefaultSpotweld) {
            ASSERT_LT(row.source_index,physical::Model().welds().model().connection_count());
            EXPECT_EQ(physical::Model().welds().model().connections()[row.source_index].source_element_id,row.original_id);
        } else {
            ASSERT_EQ(row.kind,SourceKind::RegularJoint);ASSERT_LT(row.source_index,physical::JointSource().data().rows.size());
            EXPECT_EQ(physical::JointSource().data().rows[row.source_index].source_id,row.original_id);
        }
    }
    // Numeric range is independently derived; expected mappings are test-only.
    RecordProperty("existing_maximum",std::to_string(result.existing_maximum));
    RecordProperty("final_maximum",std::to_string(result.final_maximum));
    RecordProperty("source_digest",result.source_digest);RecordProperty("mapping_digest",result.mapping_digest);
}
TEST(NativeSpringIdsActual, CopiedHandlesAndOneByteShortReservationPreserveSourceIdentity) {
    ASSERT_NO_FATAL_FAILURE(ReadySources());
    const auto copied=ActualImportContext();const auto& beams=physical::Inputs().beams;
    const auto& welds=physical::Model().welds();const auto& joints=physical::JointSource();
    const auto forecast=Preflight(beams,welds,joints,copied);
    auto cap=Limits{};cap.host_bytes=forecast.total_bytes-1;
    const auto rejected=Resolve(beams,welds,joints,copied,cap);
    EXPECT_EQ(rejected.diagnostic.status,Readiness::ResourceLimit);EXPECT_TRUE(rejected.rows.empty());
    EXPECT_EQ(Resolve(beams,welds,joints,copied).mapping_digest,ActualResolution().mapping_digest);
    EXPECT_EQ(&copied.canonical().data(),&ActualImportContext().canonical().data());
    RecordProperty("complete_source_reservation",std::to_string(forecast.total_bytes));
}
TEST(NativeSpringIdsActual, MutatedObservedEvidenceCannotBecomeAnExpectedMap) {
    const auto directory=ObservedDirectory();const auto accepted=LoadObserved(directory);
    Temporary temporary;
    auto generated=output::ReadBounded(directory/"generated-springs.csv",217584);
    const auto native=output::ReadBounded(directory/"native-spring-table.csv",1229110);
    output::WriteBytes(temporary.path/"native-spring-table.csv",native);
    generated[0]=generated[0]=='x'?'y':'x';output::WriteBytes(temporary.path/"generated-springs.csv",generated);
    EXPECT_THROW(LoadObserved(temporary.path),std::exception);
    {std::ofstream changed(temporary.path/"generated-springs.csv",std::ios::binary|std::ios::trunc);changed.write(generated.data(),std::streamsize(generated.size()-1));}
    EXPECT_THROW(LoadObserved(temporary.path),std::exception);
    EXPECT_EQ(LoadObserved(directory).generated.size(),accepted.generated.size());
}
} // namespace crash::modelio::native_spring_ids::test
