#pragma once
#include "../Internal.h"
#include "../../tied_classification/Internal.h"
#include "../../tied_finalization/tests/Fixture.h"
namespace crash::cases::vehicle_startup::post_kinchk_test {
struct Fixture : finalization_test::Fixture {
    tied::ClassificationSourceReceipt context;
    native_search::FinalizedSearch finalized;
    TiedClassificationData classified;
    TiedPostKinChkReceipt receipt;
    Fixture() {
        const auto packed = Inputs();
        if (!native_search::FinalizeSearch(packed.View(geometry),&finalized)) throw std::runtime_error("Tiny finalization failed");
        auto input = tied_classification_detail::Pack(canonical,declaration,geometry,*finalized.data(),Receipt());
        native_search::ClassificationResult result;
        if (!native_search::Classify(input.View(),&result)) throw std::runtime_error("Tiny classification failed");
        classified = tied_classification_detail::Observed(result,declaration,*finalized.data());
        context.original_type2_interfaces = 1;
        context.observed_slaves = declaration.slave_nodes.size();
        context.replaced_primitive_walls = 1;
        context.wall.part_id = 1001;
        context.wall.node_ids = {1001};
        context.wall.shell_ids = {1001};
        const auto& source = declaration.sources[declaration.contact_source].block;
        tied::UnresolvedBlock contact{source.filename,source.keyword,source.sha256,source.first_line,source.last_line};
        context.roles = {{contact,tied::ClassificationSourceRole::SelectedType2},
                         {{},tied::ClassificationSourceRole::ReplacedOriginalWall}};
        receipt = post_kinchk_detail::Receipt(canonical,declaration,Receipt(),context);
    }
};
}
