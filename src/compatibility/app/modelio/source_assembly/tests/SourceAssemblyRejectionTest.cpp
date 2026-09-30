#include "AssemblyTestSupport.h"

namespace crash::modelio::assembly::test {
namespace {
void Reject(const std::function<void(output::Document&)>& edit) {
    Scratch scratch; const auto bytes = Alter(edit);
    EXPECT_THROW(SourceAssembly::Read(scratch.Write(bytes), ExplicitTestIdentity(bytes)), std::runtime_error);
}
}  // namespace
TEST(SourceAssemblyRejection,ExpectedIdentityIsMandatoryAndChangedBytesAreRejected) {
    Scratch scratch; auto bytes = FixtureBytes(); const auto path = scratch.Write(bytes);
    EXPECT_THROW(SourceAssembly::Read(path, {}), std::runtime_error);
    auto bad_hash = PinnedYarisSixPartInventory(); bad_hash.sha256[0] = bad_hash.sha256[0] == 'a' ? 'b' : 'a';
    EXPECT_THROW(SourceAssembly::Read(path, bad_hash), std::runtime_error);
    bytes.back() = bytes.back() == ' ' ? '\n' : ' ';
    EXPECT_THROW(SourceAssembly::Read(scratch.Write(bytes), PinnedYarisSixPartInventory()), std::runtime_error);
    bytes.pop_back(); EXPECT_THROW(SourceAssembly::Read(scratch.Write(bytes), PinnedYarisSixPartInventory()), std::runtime_error);
}
TEST(SourceAssemblyRejection,DuplicateJsonKeysAndScopeEscalationAreRejected) {
    Scratch scratch; auto bytes = FixtureBytes(); const auto opening = bytes.find('{');
    bytes.insert(opening + 1, "\"schema\":\"robo-dyna.source-assembly-inventory.v1\",");
    EXPECT_THROW(SourceAssembly::Read(scratch.Write(bytes), ExplicitTestIdentity(bytes)), std::runtime_error);
    Reject([](auto& d) { d["simulation_ready"].SetBool(true); });
    Reject([](auto& d) { d["native_mass_ledger"]["frontier_only_nodes_are_physical_owner_nodes"].SetBool(true); });
    Reject([](auto& d) { d["boundary"]["applied_to_dynamics"].SetBool(true); });
}
TEST(SourceAssemblyRejection,LateGeometryAndParentMappingFailuresDoNotPublishAReplacement) {
    const auto original = Load(); const auto* nodes = original.data().nodes.data();
    Reject([](auto& d) { auto& n = d["geometry"]["parts"][5]["nodes"]; n[n.Size()-1]["source_id"].SetUint64(n[0]["source_id"].GetUint64()); });
    Reject([](auto& d) { auto& p = d["parent_bindings"]; p[p.Size()-1]["family_index"].SetUint64(0); });
    Reject([](auto& d) { auto& p = d["parent_bindings"]; p[p.Size()-1]["material_index"].SetUint64(0); });
    Reject([](auto& d) { auto& p = d["parent_bindings"]; p[p.Size()-1]["node_indices"][0].SetUint64(0); });
    Reject([](auto& d) { auto& p = d["geometry"]["parts"][5]["shells"]; p[p.Size()-1]["source_id"].SetUint64(p[0]["source_id"].GetUint64()); });
    EXPECT_EQ(original.data().nodes.data(), nodes);
    EXPECT_EQ(original.data().identity.sha256, PinnedYarisSixPartInventory().sha256);
}
TEST(SourceAssemblyRejection,SourceMaterialCurveAndElformAssociationsAreStrict) {
    Reject([](auto& d) { d["declarations"]["materials"][5]["material_id"].SetUint64(2000119); });
    Reject([](auto& d) { d["declarations"]["materials"][5]["rate_coefficient_per_s"].SetDouble(0); });
    Reject([](auto& d) { d["declarations"]["materials"][5]["cards"][1]["blank_field_mask"].SetUint(255); });
    Reject([](auto& d) { d["declarations"]["curves"][1]["plastic_strain"][45].SetDouble(.1); });
    Reject([](auto& d) { d["declarations"]["sections"][5]["source_elform"].SetUint(1); });
    Reject([](auto& d) { d["declarations"]["sections"][5]["thickness_m"][3].SetDouble(.005); });
}
TEST(SourceAssemblyRejection,LateGroupMembersAndReleasedFrontierCannotBeSilentlyChanged) {
    Reject([](auto& d) { auto& groups = d["attachments"]["nodal_rigid_groups"]; groups[7]["node_set"]["members"][15].SetUint64(123); });
    Reject([](auto& d) { auto& members = d["attachments"]["nodal_rigid_groups"][7]["node_set"]["members"]; members[15].SetUint64(members[0].GetUint64()); });
    Reject([](auto& d) { auto& groups = d["attachments"]["nodal_rigid_groups"]; groups[7]["rigid"]["identity"].SetUint64(groups[2]["rigid"]["identity"].GetUint64()); });
    Reject([](auto& d) { auto& groups = d["attachments"]["nodal_rigid_groups"]; groups[7]["rigid"]["cards"][0]["blank_mask"].SetUint(248); });
    Reject([](auto& d) { d["boundary"]["outgoing_nodal_rigid_ids"].PopBack(); });
    Reject([](auto& d) { d["boundary"]["outgoing_spotweld_ids"].PopBack(); });
    Reject([](auto& d) { d["boundary"]["released_external_tied_source_scopes"][0]["actual_pairing_qualified"].SetBool(true); });
}
TEST(SourceAssemblyRejection,EachDeclaredAdmissionCapIsChecked) {
    for (unsigned failure = 0; failure < 11; ++failure) {
        ReadLimits limits;
        if (failure == 0) limits.bytes = 100;
        if (failure == 1) limits.nodes = 1029;
        if (failure == 2) limits.parents = 914;
        if (failure == 3) limits.parts = 5;
        if (failure == 4) limits.tables = 5;
        if (failure == 5) limits.curve_points = 45;
        if (failure == 6) limits.groups = 9;
        if (failure == 7) limits.group_members = 15;
        if (failure == 8) limits.external_nodes = 36;
        if (failure == 9) limits.spotwelds = 12;
        if (failure == 10) limits.curve_points = 62; // 17 + 46 points require 63 total slots.
        EXPECT_THROW(SourceAssembly::Read(FixturePath(), PinnedYarisSixPartInventory(), limits), std::runtime_error);
    }
}
}  // namespace crash::modelio::assembly::test
