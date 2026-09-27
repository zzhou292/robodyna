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
    // Complete authenticated closure: four combine.key planar blocks and two wall.key finite blocks.
    EXPECT_EQ(source.provenance().interfaces.rigid_wall_sources, 6u);
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
TEST(CorrectedNodalSourceActual, MaterialQueriesRetainSourceIdentityUnitsAndExplicitUnavailability) {
    const auto prepared = CorrectedNodalSource::Prepare(Before(), Members().Input());
    ASSERT_EQ(prepared.report.status, Status::Ready) << prepared.report.reason;
    ASSERT_TRUE(prepared.source);
    const auto source = *prepared.source;
    const auto before = source.provenance().certificate_digest;
    const auto& parts = Before().physical().source_domain().source().solid_source().data().parts;
    ASSERT_FALSE(parts.empty());
    for (const auto& part : parts) {
        const auto query = source.material_slots(part.id);
        ASSERT_EQ(query.status, MaterialSlotStatus::Ready) << part.id;
        ASSERT_TRUE(query.values);
        const auto& slots = *query.values;
        EXPECT_EQ(slots.part_id, part.id);
        EXPECT_EQ(slots.section_id, part.section_id);
        EXPECT_EQ(slots.material_id, part.material_id);
        EXPECT_EQ(output::Bits(slots.units.length_m), output::Bits(Before().provenance().units.length_m));
        EXPECT_EQ(output::Bits(slots.units.mass_kg), output::Bits(Before().provenance().units.mass_kg));
        EXPECT_EQ(output::Bits(slots.units.time_s), output::Bits(Before().provenance().units.time_s));
        EXPECT_TRUE(std::isfinite(slots.pm32) && slots.pm32 >= 0);
        EXPECT_TRUE(std::isfinite(slots.pm100) && slots.pm100 >= 0);
        EXPECT_TRUE(std::isfinite(slots.pm107) && slots.pm107 >= 0);
        const auto copied = source.material_slots(part.id);
        ASSERT_TRUE(copied.values);
        EXPECT_EQ(output::Bits(copied.values->pm32), output::Bits(slots.pm32));
        EXPECT_EQ(output::Bits(copied.values->pm100), output::Bits(slots.pm100));
        EXPECT_EQ(output::Bits(copied.values->pm107), output::Bits(slots.pm107));
    }
    std::size_t unavailable = 0;
    for (const auto& part : source.part_controls()) {
        const auto retained = std::find_if(parts.begin(), parts.end(),
            [&](const auto& value) { return value.id == part.part_id; });
        if (retained != parts.end()) continue;
        const auto query = source.material_slots(part.part_id);
        EXPECT_EQ(query.status, MaterialSlotStatus::NotRetainedSolidPart);
        EXPECT_FALSE(query.values);
        ++unavailable;
    }
    EXPECT_GT(unavailable, 0u);
    for (const auto id : {std::uint64_t{0}, UINT64_MAX}) {
        const auto query = source.material_slots(id);
        EXPECT_EQ(query.status, MaterialSlotStatus::UnknownPart);
        EXPECT_FALSE(query.values);
    }
    EXPECT_EQ(source.provenance().certificate_digest, before);
    RecordProperty("queried_retained_parts", std::to_string(parts.size()));
    RecordProperty("unavailable_nonretained_parts", std::to_string(unavailable));
}
TEST(CorrectedNodalSourceActual, SharedControlAuthorityRetainsBackingAndChecksExactResultCaps) {
    const auto& direct=Before().solid_control_declarations();
    const auto& canonical=direct.canonical();
    const auto imported=modelio::native_spring_ids::ImportContext::Prepare(canonical,Members().Input());
    namespace control=modelio::solid_control;
    auto selected=control::EffectiveSource::Prepare(direct,imported,Members().Input());
    ASSERT_EQ(selected.report.status,control::Status::Ready)<<selected.report.reason;
    ASSERT_TRUE(selected.source);
    ASSERT_EQ(selected.source->data().parts.size(),922u);
    ASSERT_EQ(selected.source->data().origins.size(),922u);
    for(std::size_t i=0;i<922;++i) {
        EXPECT_EQ(selected.source->data().parts[i].part_id,selected.source->data().origins[i].part_id);
        EXPECT_FALSE(selected.source->data().origins[i].member.empty());
        EXPECT_EQ(selected.source->data().origins[i].block_sha256.size(),64u);
    }
    control::Limits limits;limits.retained_bytes=selected.source->owned_payload_bytes();
    EXPECT_EQ(control::EffectiveSource::Prepare(direct,imported,Members().Input(),limits).report.status,control::Status::Ready);
    --limits.retained_bytes;
    auto failed=control::EffectiveSource::Prepare(direct,imported,Members().Input(),limits);
    EXPECT_EQ(failed.report.status,control::Status::ResourceLimit);EXPECT_FALSE(failed.source);
    control::DirectLimits direct_limits;direct_limits.retained_bytes=direct.owned_payload_bytes();
    EXPECT_NO_THROW(control::DirectSource::Prepare(canonical,Members().Input(),direct_limits));
    --direct_limits.retained_bytes;
    EXPECT_THROW(control::DirectSource::Prepare(canonical,Members().Input(),direct_limits),std::exception);
    const auto clone=output::full_shell::source::CanonicalSource::ReadWithMemberBytes(canonical.data().inputs,physical::Inputs().member,canonical.data().limits);
    ASSERT_NE(&clone.data(),&canonical.data());
    const auto clone_import=modelio::native_spring_ids::ImportContext::Prepare(clone,Members().Input());
    const auto cloned=control::EffectiveSource::Prepare(direct,clone_import,Members().Input());
    ASSERT_EQ(cloned.report.status,control::Status::Ready)<<cloned.report.reason;
    ASSERT_TRUE(cloned.source);
    EXPECT_FALSE(cloned.source->data().shared_canonical_backing);
    EXPECT_EQ(cloned.source->data().additional_backing_reservation_bytes,canonical.data().limits.host_bytes);
    EXPECT_EQ(cloned.source->data().source_digest,selected.source->data().source_digest);
    EXPECT_EQ(cloned.source->data().parts.size(),selected.source->data().parts.size());
    const auto retained=*selected.source;
    selected.source.reset();
    EXPECT_EQ(retained.data().parts.size(),922u);
    EXPECT_FALSE(retained.direct().data().evidence.empty());
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_correction::test
