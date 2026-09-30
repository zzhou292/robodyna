#include "../Internal.h"
#include "../../nodal_seed/tests/ActualMembers.h"
#include "case/vehicle_startup/physical_model/tests/supports/Support.h"
#include "modelio/vehicle_sections/VehicleSectionResolution.h"
#include "lib_utest/qualification/beam18_property_area/Native.h"
#include "output/BoundedArrayJson.h"
#include <cmath>
namespace crash::cases::vehicle_self_contact::native::gap_operands::test {
namespace physical=vehicle_startup::physical_model::supports_test;
namespace {
const nodal_seed::test::ActualMembers& Members() {
    static const nodal_seed::test::ActualMembers value(physical::Model().shell_source().references().source().canonical());
    return value;
}
const source::CorrectedNodalSource& Corrected() {
    static const auto value=[] {
        const auto before=nodal_seed::PreCorrectionNodalSource::Prepare(physical::Model(),physical::Joints(),Members().Input());
        const auto result=source::CorrectedNodalSource::Prepare(before,Members().Input());
        output::Require(result.source.has_value(),"Complete corrected source fixture rejected");
        return *result.source;
    }();
    return value;
}
void SourceCounts() {
    ASSERT_EQ(Corrected().pre_correction().counts().nodes,376930u);
    ASSERT_EQ(Corrected().pre_correction().counts().shells,349645u);
    ASSERT_EQ(Corrected().pre_correction().counts().beams,142u);
}
std::filesystem::path Destination() {
    const auto* text=std::getenv("ROBO_GAP_OPERANDS_OUTPUT");
    output::Require(text&&*text,"Missing create-only gap operand output path");
    const std::filesystem::path path(text);
    output::Require(std::filesystem::create_directory(path),"Gap operand output already exists");
    return path;
}
void Write(const ContactGapOperands& source) {
    constexpr std::size_t export_limit=std::size_t{128}<<20;
    const auto binding_bytes=source.bindings().size()*6*sizeof(std::uint64_t);
    const auto shell_bytes=source.shells().size()*3*sizeof(double);
    const auto beam_bytes=source.beams().size()*2*sizeof(double);
    output::Require(2*(binding_bytes+shell_bytes+beam_bytes)+(1u<<20)<=export_limit,
        "Gap operand qualification export exceeds its separate allowance");
    const auto path=Destination();
    const output::arrays::Limits file_cap{output::kArtifactFileCap,UINT32_MAX,6};
    std::vector<std::uint64_t> bindings;bindings.reserve(source.bindings().size()*6);
    for(const auto& b:source.bindings())bindings.insert(bindings.end(),{std::uint64_t(b.family),b.original_id,b.native_id,
        b.source_part_id,b.source_index,b.operand_row});
    const auto identity=output::arrays::Write<std::uint64_t>(path,"source-bindings.bin",
        {output::arrays::Scalar::UInt64,source.bindings().size(),6,{"family","source_eid","native_eid","source_pid_or_zero","source_index","operand_row"}},
        bindings.data(),bindings.size(),file_cap);
    std::vector<double> shells;shells.reserve(source.shells().size()*3);
    for(const auto& x:source.shells())shells.insert(shells.end(),{x.part_contact_thickness,x.element_thickness,x.property_thickness});
    const auto shell_fields=output::arrays::Write<double>(path,"shell-gap-operands.bin",
        {output::arrays::Scalar::Float64,source.shells().size(),3,{"part_contact_thickness","element_thickness","property_thickness"}},
        shells.data(),shells.size(),file_cap);
    std::vector<double> beams;beams.reserve(source.beams().size()*2);
    for(const auto& x:source.beams())beams.insert(beams.end(),{x.part_contact_thickness,x.native_area});
    const auto beam_fields=output::arrays::Write<double>(path,"beam-gap-operands.bin",
        {output::arrays::Scalar::Float64,source.beams().size(),2,{"part_contact_thickness","native_pre_pmass_area"}},
        beams.data(),beams.size(),file_cap);
    output::Document doc;doc.SetObject();
    output::String(doc,"schema","robo_dyna.v5_native_gap_operands.v1");
    output::String(doc,"scope","Immutable whole retained-model operands only; mixed topology/NSV/MSR and scalar controls remain separate");
    output::String(doc,"operand_digest",source.provenance().operand_digest);
    output::String(doc,"source_digest",source.provenance().source_digest);
    output::String(doc,"contributor_digest",source.provenance().contributor_digest);
    output::Integer(doc,"nodes",source.counts().nodes);output::Integer(doc,"shells",source.counts().shells);
    output::Integer(doc,"beams",source.counts().beams);output::Integer(doc,"springs",source.springs().size());
    output::Integer(doc,"namespace_only_springs",source.counts().namespace_only_springs);
    output::Integer(doc,"solid_rows_without_direct_gap_term",source.counts().solids_without_direct_gap_term);
    output::Integer(doc,"maxima_terms",source.proof().maximum_terms);output::Integer(doc,"skipped_springs",source.proof().skipped_springs);
    output::Integer(doc,"source_peak_bytes",source.forecast().peak_bytes);output::Integer(doc,"qualification_output_limit_bytes",export_limit);
    output::Integer(doc,"qualification_peak_limit_bytes",source.forecast().peak_bytes+export_limit);
    output::Boolean(doc,"final_gaps_ready",false);output::Boolean(doc,"runtime_admitted",false);
    output::array_json::Child(doc,"bindings",output::arrays::DescriptorDocument(identity,file_cap));
    output::array_json::Child(doc,"shell_operands",output::arrays::DescriptorDocument(shell_fields,file_cap));
    output::array_json::Child(doc,"beam_operands",output::arrays::DescriptorDocument(beam_fields,file_cap));
    output::WriteJson(path/"source.json",doc);
}
}
TEST(GapOperandsActual, ForecastAndOneByteShortRejectBeforePublishing) {
    ASSERT_NO_FATAL_FAILURE(SourceCounts());
    const auto forecast=ContactGapOperands::Preflight(Corrected());
    auto exact=Limits{};exact.host_bytes=forecast.peak_bytes;
    EXPECT_EQ(ContactGapOperands::Preflight(Corrected(),exact).peak_bytes,forecast.peak_bytes);
    --exact.host_bytes;
    const auto rejected=ContactGapOperands::Prepare(Corrected(),exact);
    EXPECT_EQ(rejected.report.status,Status::ResourceLimit);EXPECT_FALSE(rejected.source);
    output::Document doc;doc.SetObject();
    output::Integer(doc,"source_peak_bytes",forecast.peak_bytes);
    output::Integer(doc,"qualification_output_limit_bytes",std::size_t{128}<<20);
    output::Integer(doc,"qualification_peak_limit_bytes",forecast.peak_bytes+(std::size_t{128}<<20));
    output::WriteJson(Destination()/"forecast.json",doc);
}
TEST(GapOperandsActual, CompletePhysicalRosterUsesNativeAreaAndSourceThickness) {
    ASSERT_NO_FATAL_FAILURE(SourceCounts());
    const auto prepared=ContactGapOperands::Prepare(Corrected());
    ASSERT_EQ(prepared.report.status,Status::Ready)<<prepared.report.reason;
    ASSERT_TRUE(prepared.source);
    const auto& source=*prepared.source;const auto& counts=source.counts();
    EXPECT_EQ(counts.shells,349645u);EXPECT_EQ(counts.quads+counts.triangles,counts.shells);EXPECT_EQ(counts.beams,142u);
    EXPECT_EQ(counts.type13,4442u);EXPECT_EQ(counts.type25,2828u);EXPECT_EQ(counts.type45,44u);
    EXPECT_EQ(counts.namespace_only_springs,35u);EXPECT_EQ(counts.solids_without_direct_gap_term,4980u);
    EXPECT_EQ(source.proof().maximum_terms,counts.shells+counts.beams);EXPECT_EQ(source.proof().skipped_springs,7314u);
    EXPECT_TRUE(source.proof().no_retained_trusses);
    const auto& references=physical::Model().shell_source().references();
    std::size_t checked_shells=0,checked_beams=0,checked_springs=0;
    for(const auto& b:source.bindings()) {
        if(b.family==Family::Quad||b.family==Family::Triangle) {
            ASSERT_LT(b.source_index,references.rows().size());ASSERT_LT(b.operand_row,source.shells().size());
            const auto& row=references.rows()[b.source_index];const auto& value=source.shells()[b.operand_row];
            EXPECT_EQ(row.element_id,b.original_id);EXPECT_EQ(row.part_id,b.source_part_id);
            const auto* section=references.resolution()->section(row.part_index);
            ASSERT_NE(section,nullptr);ASSERT_TRUE(section->cards[1].values[0]);
            EXPECT_TRUE(tl::math::SameScalarBits(value.property_thickness,*section->cards[1].values[0]));
            EXPECT_TRUE(tl::math::SameScalarBits(value.part_contact_thickness,0.));
            EXPECT_TRUE(tl::math::SameScalarBits(value.element_thickness,0.));
            ++checked_shells;
        } else if(b.family==Family::Beam18) {
            const auto* model=physical::Model().structural_beams();ASSERT_NE(model,nullptr);
            ASSERT_LT(b.source_index,model->parents().size());ASSERT_LT(b.operand_row,source.beams().size());
            const auto& parent=model->parents()[b.source_index];const auto& value=source.beams()[b.operand_row];
            const auto native=beam18_property_test::Native(parent.reference.input().radius);
            EXPECT_TRUE(tl::math::SameScalarBits(value.native_area,native[1]));
            for(unsigned k=0;k<2;++k)EXPECT_EQ(value.nodes[k],parent.domain_nodes[k]);
            EXPECT_EQ(value.source_element_id,b.original_id);++checked_beams;
        } else {
            ASSERT_LT(b.operand_row,source.springs().size());const auto& value=source.springs()[b.operand_row];
            EXPECT_EQ(value.native_element_id,b.native_id);
            EXPECT_TRUE(tl::math::SameScalarBits(value.part_contact_thickness,0.));
            EXPECT_TRUE(value.property_type==13||value.property_type==25||value.property_type==45);++checked_springs;
        }
    }
    EXPECT_EQ(checked_shells,counts.shells);EXPECT_EQ(checked_beams,counts.beams);EXPECT_EQ(checked_springs,7314u);
    const auto copy=source;EXPECT_EQ(copy.shells().data(),source.shells().data());
    EXPECT_EQ(copy.provenance().operand_digest,source.provenance().operand_digest);
    Write(source);
}
}
