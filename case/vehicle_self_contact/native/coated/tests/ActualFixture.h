#pragma once
#include "../Internal.h"
#include "case/vehicle_startup/physical_model/tests/supports/Support.h"
#include <cstdlib>
namespace crash::cases::vehicle_self_contact::native::coated::test {
namespace physical = vehicle_startup::physical_model::supports_test;
inline const selection::OriginalSelection& Selection() {
    static const auto value = [] {
        const auto* auxiliary = std::getenv("ROBO_SELF_CONTACT_AUX_MEMBER");
        const auto* combine = std::getenv("ROBO_SELF_CONTACT_COMBINE_MEMBER");
        output::Require(auxiliary && combine, "Missing authenticated original contact selection members");
        return selection::OriginalSelection::Prepare(modelio::vehicle::test::Canonical(),
            output::ReadBounded(auxiliary, selection::Limits{}.auxiliary_member_bytes),
            output::ReadBounded(combine, selection::Limits{}.combine_member_bytes));
    }();
    return value;
}
inline void SourceCounts() {
    const auto& model = physical::Model();
    ASSERT_EQ(model.source_domain().domain().node_count(), 376930u);
    ASSERT_EQ(model.shell_source().references().rows().size(), 349645u);
    ASSERT_EQ(model.source_domain().source().solid_source().data().rows.size(), 4980u);
    ASSERT_EQ(Selection().data().counts.retained_shells, 337092u);
    ASSERT_EQ(Selection().canonical().data().canonical_nodes, 393165u);
}
inline void Write(const char* name, const output::Document& document) {
    const auto* path = std::getenv("ROBO_V5_COATED_OUTPUT");
    output::Require(path && *path, "Missing create-only V5 coated assessment output");
    const std::filesystem::path directory(path);
    output::Require(std::filesystem::create_directory(directory), "V5 coating output directory already exists");
    output::WriteJson(directory/name, document);
    const auto bytes = output::ReadBounded(directory/name, 1u<<20);
    output::Document receipt; receipt.SetObject();
    output::String(receipt, "schema", "robo_dyna.v5_coated_assessment_receipt.v1");
    output::String(receipt, "scope", "retained V5 source-only assessment; no runtime or original full-case equivalence");
    output::String(receipt, "file", name); output::String(receipt, "sha256", output::Sha256(bytes));
    output::Integer(receipt, "bytes", bytes.size());
    const auto& in = Selection().canonical().data().inputs;
    output::String(receipt, "canonical_manifest_sha256", in.canonical_manifest.sha256);
    output::String(receipt, "scope_report_sha256", in.scope_report.sha256);
    output::String(receipt, "source_member_sha256", in.source_member.sha256);
    output::WriteJson(directory/"manifest.json", receipt);
}
}
