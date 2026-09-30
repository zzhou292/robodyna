#include "Fixture.h"
#include <cmath>
namespace crash::cases::vehicle_startup::cin_test {
TEST(TiedCinSourceValues, DeclaredMasterIdentityAndFourNativeSlotsRemainIndependentOfWitness) {
    Fixture f;
    const auto rows = f.Packed();
    ASSERT_EQ(rows.size(),f.classified.slaves.size());
    for (std::size_t row = 0; row < rows.size(); ++row) {
        const auto rank = f.finalized.data()->selected_masters[row];
        const auto& master = f.declaration.masters[f.packing.master_rows[rank-1]];
        EXPECT_EQ(rows[row].ordered_master_rank,rank);
        EXPECT_EQ(rows[row].master_source.element_id,master.id);
        EXPECT_EQ(rows[row].master_source.part_id,f.declaration.parts[master.part_index].id);
        EXPECT_EQ(rows[row].original_nsv_row,f.finalized.data()->slaves[row]);
        if (master.arity == 3) EXPECT_EQ(rows[row].master_source_ids[2],rows[row].master_source_ids[3]);
    }
    auto geometry = f.geometry;
    // A different optional material witness cannot rename the declared IRECT
    // patch. Witness admissibility itself belongs to the immutable producer.
    geometry.matches.clear();
    geometry.properties.clear();
    const auto same = tied_cin_detail::Pack(f.canonical,f.declaration,f.packing,geometry,*f.finalized.data(),f.classified,f.post);
    EXPECT_EQ(same.back().master_source.element_id,rows.back().master_source.element_id);
    native_search::TiedCinAttachmentModel model;
    // This older source-card fixture deliberately has collinear masters. The
    // attachment model must preserve that real domain failure, not drop it.
    EXPECT_EQ(native_search::PrepareCinAttachments(f.post,f.domain,{rows.data(),rows.size()},&model).status,
              native_search::CinAttachmentStatus::ReferencePatchRejected);
    EXPECT_FALSE(model.prepared());
}
TEST(TiedCinSourceValues, StaleWorkingBitsAndLastFinalizedIdentityRejectAndRetry) {
    Fixture f;
    const auto saved = f.Packed();
    auto geometry = f.geometry;
    const auto last = f.finalized.data()->slaves.back();
    geometry.working_positions[geometry.secondary_working_nodes[last]][0] =
        std::nextafter(geometry.working_positions[geometry.secondary_working_nodes[last]][0],1e10);
    EXPECT_THROW(tied_cin_detail::Pack(f.canonical,f.declaration,f.packing,geometry,*f.finalized.data(),f.classified,f.post),std::exception);
    auto finalized = *f.finalized.data();
    finalized.selected_masters.back() = f.geometry.masters.size()+1;
    EXPECT_THROW(tied_cin_detail::Pack(f.canonical,f.declaration,f.packing,f.geometry,finalized,f.classified,f.post),std::exception);
    auto classified = f.classified;
    ++classified.slaves.back().source_node_id;
    EXPECT_THROW(tied_cin_detail::Pack(f.canonical,f.declaration,f.packing,f.geometry,*f.finalized.data(),classified,f.post),std::exception);
    EXPECT_EQ(f.Packed().back().secondary_source_id,saved.back().secondary_source_id);
}
TEST(TiedCinSourceValues, ExactBudgetChargesDecodedArraysAndSharedPostOnce) {
    Fixture f;
    TiedPostKinChkForecast prior;
    prior.retained_classification_reservation_bytes = 10000;
    prior.receipt_payload_bytes = 1000;
    const auto out = tied_cin_detail::Budget(prior,f.post,f.domain,f.canonical.canonical_nodes,1000,{});
    EXPECT_EQ(out.post_kinchk_reservation_bytes,11000+f.post.forecast().owned_payload_bytes);
    EXPECT_EQ(out.input_staging_bytes,56*f.canonical.canonical_nodes+
        f.post.slaves().count*sizeof(native_search::CinAttachmentDeclaration));
    TiedCinAttachmentLimits limits;
    limits.host_bytes = out.total_host_bytes-1;
    EXPECT_THROW(tied_cin_detail::Budget(prior,f.post,f.domain,f.canonical.canonical_nodes,1000,limits),std::exception);
    ++limits.host_bytes;
    EXPECT_EQ(tied_cin_detail::Budget(prior,f.post,f.domain,f.canonical.canonical_nodes,1000,limits).total_host_bytes,limits.host_bytes);
}
}
