#include "../Internal.h"
#include "../../nodal_seed/tests/ActualMembers.h"
#include "case/vehicle_startup/physical_model/tests/supports/Support.h"
#include "output/BoundedArrayJson.h"
#include <algorithm>
#include <cmath>

namespace crash::cases::vehicle_self_contact::native::nodal_correction::test {
namespace physical = vehicle_startup::physical_model::supports_test;
namespace {
const seed::test::ActualMembers& Members() {
    static const seed::test::ActualMembers members(physical::Model().shell_source().references().source().canonical());
    return members;
}
const seed::PreCorrectionNodalSource& Before() {
    static const auto value = seed::PreCorrectionNodalSource::Prepare(physical::Model(), physical::Joints(), Members().Input());
    return value;
}
void Write(const CorrectedNodalSource& value) {
    const auto* path = std::getenv("ROBO_CORRECTED_NODAL_OUTPUT");
    output::Require(path && *path, "Missing create-only corrected source output");
    const std::filesystem::path directory(path);
    output::Require(std::filesystem::create_directory(directory), "Corrected source output already exists");
    const auto coefficients = value.coefficients();
    const auto array = output::arrays::Write<double>(directory, "corrected-global-K.bin",
        {output::arrays::Scalar::Float64, coefficients.size(), 1, {"native_corrected_global_K"}},
        coefficients.data(), coefficients.size());
    output::Document document;
    document.SetObject();
    output::String(document, "schema", "robo_dyna.corrected_global_nodal_source.v1");
    output::String(document, "scope", "declared V5 global nodal K only; main/gap/secondary/runtime admission separate");
    output::String(document, "source_digest", value.provenance().source_digest);
    output::String(document, "pre_correction_digest", value.provenance().pre_correction_digest);
    output::String(document, "property_digest", value.provenance().property_digest);
    output::String(document, "material_digest", value.provenance().material_digest);
    output::String(document, "certificate_digest", value.provenance().certificate_digest);
    output::Integer(document, "controlled_solids", value.provenance().certificate.controlled_solids);
    output::Integer(document, "affected_nodes", value.provenance().certificate.affected_nodes);
    output::Integer(document, "peak_reservation_bytes", value.forecast().peak_bytes);
    output::array_json::Child(document, "coefficients", output::arrays::DescriptorDocument(array));
    output::WriteJson(directory / "source.json", document);
}
}
TEST(CorrectedNodalSourceActual, CompleteForecastAndOneByteShortRejectBeforePublishing) {
    const auto forecast = CorrectedNodalSource::Preflight(Before(), Members().Input());
    auto exact = Limits{};
    exact.host_bytes = forecast.peak_bytes;
    EXPECT_EQ(CorrectedNodalSource::Preflight(Before(), Members().Input(), exact).peak_bytes, forecast.peak_bytes);
    --exact.host_bytes;
    const auto failed = CorrectedNodalSource::Prepare(Before(), Members().Input(), exact);
    EXPECT_EQ(failed.report.status, Status::ResourceLimit);
    EXPECT_FALSE(failed.source);
    RecordProperty("peak_source_reservation_bytes", std::to_string(forecast.peak_bytes));
}
TEST(CorrectedNodalSourceActual, WholeSourceFactorCertificateCoversEveryControlledIncidence) {
    const auto result = CorrectedNodalSource::Prepare(Before(), Members().Input());
    ASSERT_EQ(result.report.status, Status::Ready) << result.report.reason;
    ASSERT_TRUE(result.source);
    const auto& source = *result.source;
    const auto& certificate = source.provenance().certificate;
    EXPECT_EQ(certificate.node_count, 376930u);
    EXPECT_EQ(certificate.solid_count, 4980u);
    EXPECT_EQ(certificate.controlled_solids, 3686u);
    EXPECT_EQ(certificate.affected_nodes, 6656u);
    EXPECT_EQ(source.part_controls().size(), 922u);
    EXPECT_EQ(source.provenance().interfaces.disposition, InterfaceDisposition::CompleteNoApplicableType24);
    EXPECT_EQ(source.provenance().interfaces.type25_sources, 1u);
    EXPECT_EQ(source.provenance().interfaces.type2_sources, 1u);
    EXPECT_EQ(source.provenance().interfaces.interior_sources, 1u);
    EXPECT_EQ(source.provenance().interfaces.rigid_wall_sources, 3u);
    std::vector<unsigned char> affected(certificate.node_count, 0);
    const auto& domain = Before().physical().source_domain().domain();
    for (const auto& solid : Before().physical().source_domain().source().solid_source().data().rows) {
        const auto control = std::lower_bound(source.part_controls().begin(), source.part_controls().end(), solid.part_id,
            [](const auto& part, auto id) { return part.part_id < id; });
        ASSERT_NE(control, source.part_controls().end());
        ASSERT_EQ(control->part_id, solid.part_id);
        if (!control->effective_control) continue;
        // Independent original source NID incidence, not certificate tags or
        // the packed correction raw-slot array. Permutation does not change the set.
        for (const auto id : solid.raw_node_ids) {
            const auto node = domain.Find(id);
            ASSERT_LT(node, affected.size());
            affected[node] = 1;
        }
    }
    std::size_t changed = 0;
    ASSERT_EQ(source.coefficients().size(), Before().fields().size());
    for (std::size_t i = 0; i < affected.size(); ++i) {
        const auto actual = source.coefficients()[i];
        const auto prior = Before().fields()[i].stiffness_before_control;
        ASSERT_TRUE(std::isfinite(actual));
        if (affected[i]) {
            EXPECT_GT(actual, prior);
            ++changed;
        } else {
            EXPECT_EQ(output::Bits(actual), output::Bits(prior));
        }
    }
    EXPECT_EQ(changed, 6656u);
    EXPECT_EQ(source.provenance().pre_correction_digest, Before().provenance().contributor_digest);
    EXPECT_TRUE(source.pre_correction().physical().SharesStorage(physical::Model()));
    RecordProperty("scope", "complete V5 corrected global nodal source; no runtime or main/gap admission");
    RecordProperty("certificate_digest", source.provenance().certificate_digest);
    Write(source);
}
TEST(CorrectedNodalSourceActual, ForeignMemberRejectsWithoutChangingTheImmutableInput) {
    auto members = Members().Input();
    std::string changed(members.members[0].bytes);
    changed[0] = changed[0] == '*' ? '$' : '*';
    members.members[0].bytes = changed;
    const auto digest = Before().provenance().contributor_digest;
    const auto first = Before().fields()[0].stiffness_before_control;
    const auto failed = CorrectedNodalSource::Prepare(Before(), members);
    EXPECT_NE(failed.report.status, Status::Ready);
    EXPECT_FALSE(failed.source);
    EXPECT_EQ(Before().provenance().contributor_digest, digest);
    EXPECT_EQ(output::Bits(Before().fields()[0].stiffness_before_control), output::Bits(first));
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_correction::test
