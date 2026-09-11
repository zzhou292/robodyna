#include "ReferenceSourceSupport.h"
#include <cmath>

namespace crash::cases::vehicle_startup::test {
TEST(VehicleReferenceSource, EveryOriginalParentRetainsNativeStatusAndExactSourceInputs) {
    const auto& r=References();const auto& s=r.source();const auto& c=r.counts();
    ASSERT_EQ(r.rows().size(),349645);EXPECT_EQ(c.attempted,278301);EXPECT_EQ(c.unresolved,71344);
    EXPECT_EQ(c.qeph_attempted,259200);EXPECT_EQ(c.t3_attempted,19101);
    EXPECT_EQ(c.succeeded+c.rejected,c.attempted);EXPECT_EQ(c.qeph_succeeded+c.t3_succeeded,c.succeeded);
    const detail::Geometry g(s.canonical().data());std::size_t first_error=SIZE_MAX;
    for(std::size_t i=0;i<r.rows().size();++i) {
        const auto& row=r.rows()[i];const auto p=s.parents()[i];
        ASSERT_EQ(row.canonical_parent,p.canonical_parent);ASSERT_EQ(row.part_index,p.part_index);
        ASSERT_EQ(row.element_id,g.records[6*p.canonical_parent]);ASSERT_EQ(row.part_id,g.records[6*p.canonical_parent+1]);
        ASSERT_EQ(row.source_line,g.lines[p.canonical_parent]);
        ASSERT_EQ(row.material_id,s.parts()[p.part_index].material_id);ASSERT_EQ(row.section_id,s.parts()[p.part_index].section_id);
        if(s.parts()[p.part_index].status==modelio::vehicle::Disposition::Unresolved) {
            ASSERT_EQ(row.status,ReferenceStatus::UnresolvedDeclaration);ASSERT_EQ(row.family,ReferenceFamily::None);
            ASSERT_EQ(r.qeph(i),nullptr);ASSERT_EQ(r.t3(i),nullptr);continue;
        }
        const bool quad=g.records[6*p.canonical_parent+4]!=g.records[6*p.canonical_parent+5];
        ASSERT_EQ(row.family,quad?ReferenceFamily::Qeph:ReferenceFamily::T3);
        if(row.status!=ReferenceStatus::Success) {
            if(first_error==SIZE_MAX)first_error=i;
            ASSERT_EQ(row.reference_index,SIZE_MAX);ASSERT_EQ(r.qeph(i),nullptr);ASSERT_EQ(r.t3(i),nullptr);continue;
        }
        if(quad) {ASSERT_NE(r.qeph(i),nullptr);ASSERT_EQ(r.t3(i),nullptr);CheckSourceInput(r.qeph(i)->input,row,g);CheckNative(*r.qeph(i));}
        else {ASSERT_NE(r.t3(i),nullptr);ASSERT_EQ(r.qeph(i),nullptr);CheckSourceInput(r.t3(i)->input,row,g);CheckNative(*r.t3(i));}
    }
    EXPECT_EQ(r.qeph(r.rows().size()),nullptr);EXPECT_EQ(r.t3(SIZE_MAX),nullptr);
    EXPECT_EQ(r.first_error(),first_error==SIZE_MAX?nullptr:&r.rows()[first_error]);
    RecordProperty("native_attempted",std::to_string(c.attempted));RecordProperty("native_succeeded",std::to_string(c.succeeded));
    RecordProperty("native_rejected",std::to_string(c.rejected));RecordProperty("unresolved",std::to_string(c.unresolved));
    for(std::size_t i=0;i<c.status.size();++i)RecordProperty(Name(static_cast<ReferenceStatus>(i)),std::to_string(c.status[i]));
    if(const auto* row=r.first_error()) {
        RecordProperty("first_native_error_EID",std::to_string(row->element_id));RecordProperty("first_native_error_PID",std::to_string(row->part_id));
        RecordProperty("first_native_error_line",std::to_string(row->source_line));RecordProperty("first_native_error_status",Name(row->status));
        RecordProperty("first_native_error_family",row->family==ReferenceFamily::Qeph?"QEPH":"T3");
    }
    RecordProperty("source_declaration_sha256",s.identity().sha256);
    RecordProperty("forecast_bytes",std::to_string(r.forecast().total_bytes));
    RecordProperty("source_bound_bytes",std::to_string(r.forecast().source_bound_bytes));
    RecordProperty("reference_capacity_bytes",std::to_string(r.forecast().reference_capacity_bytes));
    RecordProperty("row_bytes",std::to_string(r.forecast().row_bytes));
    RecordProperty("decode_bytes",std::to_string(r.forecast().decode_bytes));
    RecordProperty("decode_temporary_bytes",std::to_string(r.forecast().decode_temporary_bytes));
}
TEST(VehicleReferenceSource, ForecastBeforeAllocatingPreservesImmutableAssessmentAndExactRetry) {
    const auto& r=References();auto copy=r;auto moved=std::move(copy);
    EXPECT_EQ(copy.rows().data(),r.rows().data());EXPECT_EQ(moved.rows().data(),r.rows().data());
    EXPECT_EQ(&moved.source().canonical().data(),&r.source().canonical().data());
    const auto& source=r.source();const auto original=r.rows().back();
    for(unsigned fault=0;fault<7;++fault) {
        ReferenceLimits limits;
        if(fault==0)limits.parents=source.counts().parents-1;
        if(fault==1)limits.nodes=source.counts().nodes-1;
        if(fault==2)limits.host_bytes=r.forecast().total_bytes-1;
        if(fault==3)limits.parents=524289;
        if(fault==4)limits.nodes=524289;
        if(fault==5)limits.host_bytes=SIZE_MAX;
        if(fault==6)limits.parents=0;
        EXPECT_THROW(VehicleShellReferences::Prepare(source,limits),std::runtime_error);
        EXPECT_EQ(r.rows().back().element_id,original.element_id);EXPECT_EQ(r.rows().back().status,original.status);
    }
    ReferenceLimits exact;exact.parents=source.counts().parents;exact.nodes=source.counts().nodes;
    exact.host_bytes=r.forecast().total_bytes;
    EXPECT_EQ(ForecastReferences(source,exact).total_bytes,r.forecast().total_bytes);
    const auto retry=VehicleShellReferences::Prepare(source,exact);
    ASSERT_EQ(retry.rows().size(),r.rows().size());EXPECT_EQ(retry.counts().status,r.counts().status);
    for(std::size_t i=0;i<r.rows().size();++i) {
        ASSERT_EQ(retry.rows()[i].element_id,r.rows()[i].element_id);ASSERT_EQ(retry.rows()[i].status,r.rows()[i].status);
        if(const auto* q=r.qeph(i))Same(*q,*retry.qeph(i));
        if(const auto* t=r.t3(i))Same(*t,*retry.t3(i));
    }
}
} // namespace crash::cases::vehicle_startup::test
