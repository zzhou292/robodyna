#include "../Internal.h"
#include "../../nodal_seed/tests/ActualMembers.h"
#include "../../coated/tests/ActualFixture.h"
#include "output/BoundedArrayJson.h"
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::main_coefficients::test {
namespace physical = vehicle_startup::physical_model::supports_test;
namespace seed = nodal_seed;
namespace {
const seed::test::ActualMembers& Members() {
    static const seed::test::ActualMembers members(physical::Model().shell_source().references().source().canonical());
    return members;
}
const c::CorrectedNodalSource& Corrected() {
    static const auto source = [] {
        const auto pre = seed::PreCorrectionNodalSource::Prepare(physical::Model(), physical::Joints(), Members().Input());
        const auto corrected = c::CorrectedNodalSource::Prepare(pre, Members().Input());
        output::Require(corrected.source.has_value(), "Actual main-source fixture lacks corrected source context");
        return *corrected.source;
    }();
    return source;
}
const std::string& Combine() {
    static const auto bytes = modelio::native_spring_ids::test::FixtureMember("ROBO_SELF_CONTACT_COMBINE_MEMBER", 64u<<10);
    return bytes;
}
const auto& Selection() { return coated::test::Selection(); }
struct QualificationBudget {
    static constexpr std::size_t OutputLimit = std::size_t{256} << 20;
    std::size_t source = 0, output = 0, total = 0;
};
QualificationBudget WorkingBytes(const Forecast& forecast) {
    // Export-only arrays and encodings are a separate qualification allowance;
    // the unchanged production source still admits at most 8 GiB. Both remain
    // inside the existing 10 GiB process guard; this does not change that guard.
    const auto p = Selection().data().counts.retained_shells;
    constexpr auto per_primary = 4*(13*sizeof(std::uint64_t)+2*sizeof(double));
    constexpr std::size_t fixed = 1u << 20;
    output::Require(p <= (QualificationBudget::OutputLimit-fixed)/per_primary,
        "Qualification export exceeds its separate 256 MiB allowance");
    const auto extra = p*per_primary+fixed;
    output::Require(forecast.peak_bytes <= Limits{}.host_bytes &&
        extra <= QualificationBudget::OutputLimit && forecast.peak_bytes <= SIZE_MAX-extra,
        "Whole-source qualification source/output reservation is invalid");
    const auto total = forecast.peak_bytes+extra;
    output::Require(total <= Limits{}.host_bytes+QualificationBudget::OutputLimit,
        "Combined source/export qualification exceeds 8 GiB plus 256 MiB");
    return {forecast.peak_bytes, extra, total};
}
void RecordBudget(output::Document& document, const QualificationBudget& budget) {
    output::Integer(document, "source_peak_bytes", budget.source);
    output::Integer(document, "qualification_output_bytes", budget.output);
    output::Integer(document, "qualification_peak_bytes", budget.total);
    output::Integer(document, "source_limit_bytes", Limits{}.host_bytes);
    output::Integer(document, "qualification_output_limit_bytes", QualificationBudget::OutputLimit);
    output::Integer(document, "qualification_limit_bytes", Limits{}.host_bytes+QualificationBudget::OutputLimit);
}

std::filesystem::path Destination() {
    const auto* text = std::getenv("ROBO_SELECTED_MAIN_OUTPUT");
    output::Require(text && *text, "Missing create-only selected main source output");
    const std::filesystem::path path(text);
    output::Require(std::filesystem::create_directory(path), "Selected main source destination already exists");
    return path;
}
void Write(const SelectedShellMainSource& source) {
    const auto budget = WorkingBytes(source.forecast());
    const auto path=Destination();
    const auto values=source.coefficients();
    const auto k=output::arrays::Write<double>(path,"expanded-main-K.bin",
        {output::arrays::Scalar::Float64,values.size(),1,{"native_K"}}, values.data(), values.size());
    std::vector<std::uint64_t> bindings;
    bindings.reserve(source.bindings().size()*13);
    for (const auto& b : source.bindings()) bindings.insert(bindings.end(), {b.contact_element,b.contact_part,
        b.support_element,b.support_part,b.solid_element,b.solid_part,b.contact_physical,b.support_physical,
        b.winner_begin,b.winner_count,b.partner,std::uint64_t(b.role),std::uint64_t(b.owner)});
    const auto binding=output::arrays::Write<std::uint64_t>(path,"primary-source-bindings.bin",
        {output::arrays::Scalar::UInt64,source.bindings().size(),13,{"contact_eid","contact_pid","support_eid","support_pid",
            "solid_eid","solid_pid","contact_physical","support_physical","winner_begin","winner_count","partner","role","owner_proof"}},
        bindings.data(),bindings.size());
    std::vector<std::uint64_t> owners;
    owners.reserve(source.possible_owners().size()*3);
    for (const auto& owner : source.possible_owners()) owners.insert(owners.end(), {owner.element,owner.part,owner.physical});
    const auto owner_array=output::arrays::Write<std::uint64_t>(path,"candidate-support-owners.bin",
        {output::arrays::Scalar::UInt64,source.possible_owners().size(),3,{"eid","pid","physical_parent"}},owners.data(),owners.size());
    const auto& cert=source.certificate();
    output::Document doc; doc.SetObject();
    output::String(doc,"schema","robo_dyna.selected_shell_main_source.v1");
    output::String(doc,"scope","Complete selected V5 shell main coefficients; no solid-face/gap/runtime admission");
    output::String(doc,"phase","Post shell grouping and BUILD_CNEL; SH2 primaries certified unchanged through pre-INITIA I25GAPM");
    output::String(doc,"coordinate_authority","Closed direct-key unprojected source; mechanical NLOC remains distinct");
    output::String(doc,"coefficient_digest",source.provenance().coefficient_digest);
    output::String(doc,"topology_digest",source.provenance().topology_digest);
    output::String(doc,"source_digest",source.provenance().import_digest);
    output::String(doc,"property_digest",source.provenance().property_digest);
    output::String(doc,"material_digest",source.provenance().material_digest);
    output::Integer(doc,"primaries",cert.primaries); output::Integer(doc,"expanded_mains",values.size());
    output::Integer(doc,"coated",cert.coated); output::Integer(doc,"negative_support_volumes",cert.negative_support_volumes);
    output::Integer(doc,"unique_owners",cert.unique_owners); output::Integer(doc,"corner_owners",cert.corner_owners);
    output::Integer(doc,"material_group_owners",cert.material_group_owners); output::Integer(doc,"unresolved_owners",cert.unresolved_owners);
    output::Integer(doc,"physical_duplicate_groups",cert.physical_duplicate_groups);
    output::Integer(doc,"physical_duplicate_rows",cert.physical_duplicate_rows);
    output::Integer(doc,"topology_warnings",source.topology_report().neighbor_warnings.count);
    RecordBudget(doc, budget);
    output::Integer(doc,"peak_reservation_bytes",budget.total);
    output::Boolean(doc,"orientation_identity",cert.orientation_identity); output::Boolean(doc,"owners_complete",cert.owners_complete);
    output::Boolean(doc,"grouping_controls_certified",cert.grouping_controls_certified);
    output::Boolean(doc,"runtime_admitted",false);
    output::array_json::Child(doc,"coefficients",output::arrays::DescriptorDocument(k));
    output::array_json::Child(doc,"bindings",output::arrays::DescriptorDocument(binding));
    output::array_json::Child(doc,"candidate_owners",output::arrays::DescriptorDocument(owner_array));
    output::WriteJson(path/"source.json",doc);
}
}
TEST(SelectedShellMainActual, ForecastAndExactCapRejectBeforePublishing) {
    ASSERT_NO_FATAL_FAILURE(coated::test::SourceCounts());
    const auto forecast=SelectedShellMainSource::Preflight(Corrected(),Selection());
    auto cap=Limits{}; cap.host_bytes=forecast.peak_bytes;
    EXPECT_EQ(SelectedShellMainSource::Preflight(Corrected(),Selection(),cap).peak_bytes,forecast.peak_bytes);
    --cap.host_bytes;
    const auto rejected=SelectedShellMainSource::Prepare(Corrected(),Selection(),physical::Inputs().member,Combine(),cap);
    EXPECT_EQ(rejected.report.status,Status::ResourceLimit);
    EXPECT_FALSE(rejected.source);
    output::Document doc;doc.SetObject();
    RecordBudget(doc, WorkingBytes(forecast));
    output::Integer(doc,"topology_output_bytes",forecast.coating.topology.output_bytes);
    output::Integer(doc,"topology_scratch_bytes",forecast.coating.topology.scratch_bytes);
    output::WriteJson(Destination()/"forecast.json",doc);
}
TEST(SelectedShellMainActual, CompleteSourceHasEveryCoefficientAndIndependentlyCertifiedOwner) {
    ASSERT_NO_FATAL_FAILURE(coated::test::SourceCounts());
    const auto prepared=SelectedShellMainSource::Prepare(Corrected(),Selection(),physical::Inputs().member,Combine());
    ASSERT_EQ(prepared.report.status,Status::CoefficientValuesReady) << prepared.report.reason;
    ASSERT_TRUE(prepared.source);
    const auto copied=*prepared.source;
    const auto& source=*prepared.source;
    const auto& cert=source.certificate();
    EXPECT_EQ(cert.primaries,337092u); EXPECT_EQ(source.coefficients().size(),674184u);
    EXPECT_EQ(cert.coated,741u); EXPECT_EQ(cert.negative_support_volumes,36u);
    EXPECT_EQ(cert.physical_duplicate_groups,4251u); EXPECT_EQ(cert.physical_duplicate_rows,12753u);
    EXPECT_TRUE(cert.orientation_identity); EXPECT_TRUE(cert.grouping_controls_certified);
    EXPECT_TRUE(cert.owners_complete); EXPECT_EQ(cert.unresolved_owners,0u);
    EXPECT_EQ(source.provenance().topology_digest,"0c8c4cfdebcbca4cb10a4d31733023e518e290d2727eabe675d7b7ae40b94beb");
    EXPECT_EQ(copied.coefficients().data(),source.coefficients().data());
    EXPECT_EQ(&source.selection().canonical().data(), &Selection().canonical().data());
    const auto& rows=physical::Model().shell_source().references().rows();
    std::size_t different=0;
    for (const auto& b : source.bindings()) {
        ASSERT_LT(b.contact_physical,rows.size());
        EXPECT_EQ(rows[b.contact_physical].element_id,b.contact_element);
        ASSERT_LT(b.support_physical,rows.size());
        EXPECT_EQ(rows[b.support_physical].element_id,b.support_element);
        EXPECT_EQ(rows[b.support_physical].part_id,b.support_part);
        different+=b.contact_element!=b.support_element;
        ASSERT_GT(b.partner,source.bindings().size());
        ASSERT_LE(b.partner,source.coefficients().size());
    }
    EXPECT_EQ(different,4251u);
    for (const auto value : source.coefficients()) EXPECT_TRUE(std::isfinite(value)&&value>0);
    Write(source);
    // Wrong control bytes and low-cap attempts never change the published handle.
    auto changed=Combine(); changed[0]=changed[0]=='*'?'$':'*';
    const auto failed=SelectedShellMainSource::Prepare(Corrected(),Selection(),physical::Inputs().member,changed);
    EXPECT_FALSE(failed.source);
    EXPECT_EQ(failed.report.status,Status::InvalidInput);
    EXPECT_EQ(source.provenance().coefficient_digest,copied.provenance().coefficient_digest);
}
}
