#include "ResolvedReferenceSupport.h"
#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_src/elements/t3/T3Startup.h"

namespace crash::cases::vehicle_startup::test {
TEST(VehicleResolvedReference, EveryOriginalRowUsesResolvedMaterialAndRetainsNativeStatus) {
    const auto& r=ResolvedReferences();
    const auto& resolution=source_test::Resolution();
    const auto& count=r.counts();
    ASSERT_EQ(r.rows().size(),349645);
    EXPECT_EQ(count.attempted,326082);
    EXPECT_EQ(count.unresolved,23563);
    EXPECT_EQ(count.succeeded+count.rejected,count.attempted);
    ASSERT_NE(r.resolution(),nullptr);
    EXPECT_EQ(r.resolution()->parents().data(),resolution.parents().data());
    EXPECT_EQ(&r.source().canonical().data(),&resolution.source().canonical().data());
    const detail::Geometry geometry(r.source().canonical().data());
    std::size_t first_error=SIZE_MAX,new_attempted=0,new_succeeded=0;
    for (std::size_t e=0;e<r.rows().size();++e) {
        const auto& row=r.rows()[e];
        const auto& original=resolution.parents()[e];
        const auto status=resolution.parts()[row.part_index].status;
        ASSERT_EQ(row.element_id,original.source_parent_id);
        ASSERT_EQ(row.canonical_parent,original.canonical_parent);
        ASSERT_EQ(row.part_index,original.part_index);
        ASSERT_EQ(row.source_line,geometry.lines[row.canonical_parent]);
        ASSERT_EQ(row.material_id,r.source().parts()[row.part_index].material_id);
        ASSERT_EQ(row.section_id,r.source().parts()[row.part_index].section_id);
        if (status==modelio::vehicle::SectionDisposition::Unresolved) {
            ASSERT_EQ(row.status,ReferenceStatus::UnresolvedDeclaration);
            ASSERT_EQ(row.family,ReferenceFamily::None);
            ASSERT_EQ(r.qeph(e),nullptr);
            ASSERT_EQ(r.t3(e),nullptr);
            continue;
        }
        const bool added=status==modelio::vehicle::SectionDisposition::ConstantFailure;
        if (added) ++new_attempted;
        if (row.status!=ReferenceStatus::Success) {
            if (first_error==SIZE_MAX) first_error=e;
            ASSERT_EQ(r.qeph(e),nullptr);
            ASSERT_EQ(r.t3(e),nullptr);
            continue;
        }
        if (added) ++new_succeeded;
        if (original.topology==modelio::vehicle::SourceShellTopology::Q4) {
            ASSERT_NE(r.qeph(e),nullptr);
            ASSERT_EQ(r.t3(e),nullptr);
            CheckResolvedInput(r.qeph(e)->input,row,geometry);
            CheckNative(*r.qeph(e));
            if (added) {
                tl::fea::qeph::ReferenceData native;
                ASSERT_EQ(tl::fea::qeph::InitializeReference(r.qeph(e)->input,native),tl::fea::qeph::Status::kSuccess);
                Same(native,*r.qeph(e));
            }
        } else {
            ASSERT_NE(r.t3(e),nullptr);
            ASSERT_EQ(r.qeph(e),nullptr);
            CheckResolvedInput(r.t3(e)->input,row,geometry);
            CheckNative(*r.t3(e));
            if (added) {
                tl::fea::t3::ReferenceData native;
                ASSERT_EQ(tl::fea::t3::InitializeReference(r.t3(e)->input,native),tl::fea::t3::Status::kSuccess);
                Same(native,*r.t3(e));
            }
        }
    }
    EXPECT_EQ(new_attempted,47781);
    EXPECT_EQ(r.first_error(),first_error==SIZE_MAX ? nullptr : &r.rows()[first_error]);
    RecordProperty("native_attempted",std::to_string(count.attempted));
    RecordProperty("native_succeeded",std::to_string(count.succeeded));
    RecordProperty("native_rejected",std::to_string(count.rejected));
    RecordProperty("qeph_attempted",std::to_string(count.qeph_attempted));
    RecordProperty("t3_attempted",std::to_string(count.t3_attempted));
    RecordProperty("new_native_succeeded",std::to_string(new_succeeded));
    RecordProperty("unresolved",std::to_string(count.unresolved));
    RecordProperty("forecast_bytes",std::to_string(r.forecast().total_bytes));
    RecordProperty("source_bound_bytes",std::to_string(r.forecast().source_bound_bytes));
    RecordProperty("reference_capacity_bytes",std::to_string(r.forecast().reference_capacity_bytes));
    RecordProperty("row_bytes",std::to_string(r.forecast().row_bytes));
    RecordProperty("fixed_bytes",std::to_string(r.forecast().fixed_bytes));
    RecordProperty("source_resolution_sha256",resolution.identity().sha256);
    if (const auto* error=r.first_error()) {
        RecordProperty("first_native_error_EID",std::to_string(error->element_id));
        RecordProperty("first_native_error_PID",std::to_string(error->part_id));
        RecordProperty("first_native_error_MID",std::to_string(error->material_id));
        RecordProperty("first_native_error_SID",std::to_string(error->section_id));
        RecordProperty("first_native_error_status",Name(error->status));
    }
}
} // namespace crash::cases::vehicle_startup::test
