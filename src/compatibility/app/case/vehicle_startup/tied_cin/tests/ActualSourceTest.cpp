#include "../Internal.h"
#include "../../tied_classification/tests/ActualClassification.h"
#include "lib_utest/qualification/tied_shell_patch/NativeOracle.h"
#include "lib_src/assembly/NodalDomainIdentity.h"
#include <iostream>
namespace crash::cases::vehicle_startup::cin_actual_test {
const TiedSearchPostKinChk& Post() {
    static const auto value = TiedSearchPostKinChk::Prepare(classification_actual_test::Prepared());
    return value;
}
const tl::fea::NodalNodeDomain& Domain() {
    static const auto value = [] {
        const auto& geometry = Post().classification().finalized().assessment().geometry();
        const auto& canonical = geometry.packing().declaration().canonical().data();
        const auto& ids_array = tied::source::FindArray(canonical,"node_ids");
        const auto& x_array = tied::source::FindArray(canonical,"node_positions");
        const auto ids = output::arrays::Decode<tied::SourceId>(ids_array.descriptor,ids_array.bytes);
        const auto x = output::arrays::Decode<double>(x_array.descriptor,x_array.bytes);
        std::vector<tl::fea::NodalDomainNode> nodes;
        // The test declares this geometric source union only. It is not a full
        // vehicle DOF inventory. Reverse order proves a real source-index map.
        for (auto it = geometry.data().canonical_nodes.rbegin(); it != geometry.data().canonical_nodes.rend(); ++it) {
            const auto row = *it;
            nodes.push_back({ids.at(row),{x.at(3*row),x.at(3*row+1),x.at(3*row+2)}});
        }
        tl::fea::NodalNodeDomain domain;
        output::Require(bool(domain.Initialize({Post().result().source_instance_id(),nodes.data(),nodes.size()},
                                             tl::fea::NodalDomainLimits::Vehicle())),"Original tied geometric domain rejected");
        return domain;
    }();
    return value;
}
const TiedCinAttachments& Prepared() {
    static const auto value = [] {
        const auto forecast = TiedCinAttachments::Forecast(Post(),Domain());
        std::cout << "Original CIN mapping preflight " << forecast.total_host_bytes << " bytes\n";
        return TiedCinAttachments::Prepare(Post(),Domain());
    }();
    return value;
}
TEST(TiedCinActual, AllOriginalRowsKeepDeclaredPatchAndMatchNativeReferenceGeometry) {
    const auto& value = Prepared();
    const auto& post = value.post_kinchk();
    const auto& finalized = post.classification().finalized();
    const auto& geometry = finalized.assessment().geometry();
    const auto& g = geometry.data();
    const auto& declaration = geometry.packing().declaration().data();
    const auto& id_array = tied::source::FindArray(geometry.packing().declaration().canonical().data(),"node_ids");
    const auto source_ids = output::arrays::Decode<tied::SourceId>(id_array.descriptor,id_array.bytes);
    const auto& model = value.model();
    ASSERT_EQ(model.rows().count,11165u);
    EXPECT_TRUE(model.domain()->SharesStorage(Domain()));
    EXPECT_TRUE(model.classification()->SharesStorage(post.result()));
    std::size_t triangles = 0;
    for (std::size_t row = 0; row < model.rows().count; ++row) {
        const auto& mapped = model.rows().data[row];
        const auto& master = finalized.selected_master(row);
        const auto rank = finalized.data().selected_masters[row];
        const auto original = finalized.data().slaves[row];
        EXPECT_EQ(mapped.original_nsv_row,original);
        EXPECT_EQ(mapped.ordered_master_rank,rank);
        EXPECT_EQ(mapped.master_source.element_id,master.id);
        EXPECT_EQ(mapped.master_source.part_id,declaration.parts[master.part_index].id);
        EXPECT_EQ(model.domain()->nodes()[mapped.secondary_domain_node].source_id,finalized.secondary(row).id);
        native_search::PatchInput patch;
        patch.secondary_position = model.domain()->nodes()[mapped.secondary_domain_node].position;
        for (std::size_t slot = 0; slot < 4; ++slot) {
            patch.master_position[slot] = model.domain()->nodes()[mapped.master_domain_nodes[slot]].position;
            const auto working = g.masters[rank-1].working_nodes[slot];
            EXPECT_EQ(model.domain()->nodes()[mapped.master_domain_nodes[slot]].source_id,
                      source_ids.at(g.canonical_nodes[working]));
            const auto& raw = g.working_positions[working];
            EXPECT_TRUE(tl::fea::nodal_domain_detail::SamePosition(patch.master_position[slot],
                {raw[0]*g.working_length_to_m,raw[1]*g.working_length_to_m,raw[2]*g.working_length_to_m}));
        }
        const bool triangle = master.arity == 3;
        triangles += triangle;
        if (triangle) EXPECT_EQ(mapped.master_domain_nodes[2],mapped.master_domain_nodes[3]);
        const auto native = tied_patch_test::Native(patch,{}, {},triangle);
        for (std::size_t c = 0; c < 7; ++c)
            tied_patch_test::Near(mapped.reference_patch.values().cofactor[c],native.cofactor[c]);
    }
    EXPECT_EQ(model.current_geometry(),native_search::CinAttachmentObligation::Pending);
    EXPECT_EQ(model.master_activity_and_release(),native_search::CinAttachmentObligation::Pending);
    RecordProperty("original_cin_attachments",std::to_string(model.rows().count));
    RecordProperty("original_triangle_attachments",std::to_string(triangles));
    RecordProperty("declared_geometric_domain_nodes",std::to_string(Domain().node_count()));
    RecordProperty("complete_cin_forecast_bytes",std::to_string(value.forecast().total_host_bytes));
    RecordProperty("cin_model_payload_bytes",std::to_string(model.forecast().model_payload_bytes));
}
TEST(TiedCinActual, LastRequiredNodeAndExactCapsFailAtomicallyWithSuccessfulRetry) {
    const auto& saved = Prepared();
    auto copy = saved;
    auto moved = std::move(copy);
    EXPECT_EQ(moved.model().rows().data,copy.model().rows().data);
    TiedCinAttachmentLimits limits;
    limits.host_bytes = saved.forecast().total_host_bytes-1;
    EXPECT_THROW(TiedCinAttachments::Prepare(Post(),Domain(),limits),std::exception);
    ++limits.host_bytes;
    EXPECT_EQ(TiedCinAttachments::Forecast(Post(),Domain(),limits).total_host_bytes,limits.host_bytes);
    const auto id = Post().classification().data().slaves.back().source_node_id;
    std::vector<tl::fea::NodalDomainNode> nodes;
    for (const auto& node : Domain().nodes()) if (node.source_id != id) nodes.push_back(node);
    tl::fea::NodalNodeDomain missing;
    ASSERT_TRUE(missing.Initialize({Post().result().source_instance_id(),nodes.data(),nodes.size()},
                                  tl::fea::NodalDomainLimits::Vehicle()));
    try {
        (void)TiedCinAttachments::Prepare(Post(),missing);
        FAIL() << "Missing final original secondary was accepted";
    } catch (const TiedCinAttachmentError& error) {
        EXPECT_EQ(error.report.status,native_search::CinAttachmentStatus::MissingDomainNode);
        EXPECT_EQ(error.report.row,11164u);
        EXPECT_EQ(error.report.slot,0u);
        EXPECT_EQ(error.source_secondary_id,id);
    }
    const auto retry = TiedCinAttachments::Prepare(Post(),Domain(),limits);
    EXPECT_EQ(retry.model().rows().count,saved.model().rows().count);
    EXPECT_EQ(retry.model().rows().data[11164].secondary_domain_node,saved.model().rows().data[11164].secondary_domain_node);
}
}
