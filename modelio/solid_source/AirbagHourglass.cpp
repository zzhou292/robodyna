#include "Internal.h"
#include "AirbagSourceReceipt.h"
#include "modelio/vehicle_source/SourceCards.h"

namespace crash::modelio::solid_source::detail {
void ReadAirbagHourglass(const Value& files, Data& data, Limits limits) {
    Require(data.policy == Policy::OriginalVehicleSupportsV5 && files.IsObject(),
            "Airbag original control proof requires the explicit V5 policy");
    std::size_t found = 0;
    for (const auto& file : files.GetObject()) {
        const auto& blocks = Array(file.value, "blocks", limits.blocks);
        std::size_t count = 0;
        for (const auto& block : blocks.GetArray()) {
            if (Text(block, "keyword") != "*CONTROL_HOURGLASS") continue;
            ++count;
            ++found;
            Require(std::string(file.name.GetString()) == "combine.key" &&
                Text(block, "file") == "combine.key" && Unsigned(block, "first_line") == 142 &&
                Unsigned(block, "last_line") == 148 && Unsigned(block, "data_records") == 1 &&
                Text(block, "source_block_sha256") == AirbagHourglassHash,
                "Original global hourglass identity changed");
        }
        const auto& inventory = Member(file.value, "keyword_counts");
        Require(inventory.IsObject(), "Invalid original global hourglass inventory");
        const auto declared = inventory.HasMember("*CONTROL_HOURGLASS") ?
            Unsigned(inventory, "*CONTROL_HOURGLASS") : 0;
        Require(count == declared, "Original global hourglass census differs from blocks");
    }
    Require(found == 1 && output::Sha256(AirbagHourglassRaw) == AirbagHourglassHash,
            "Missing or changed original global hourglass receipt");
    tied_shell::SourceEvidence evidence;
    evidence.block = {"combine.key", "*CONTROL_HOURGLASS", AirbagHourglassRaw,
                      AirbagHourglassHash, 142, 148};
    const auto cards = vehicle::detail::ReadSourceCards(evidence.block, 0, 1);
    for (const auto& card : cards) evidence.cards.emplace_back(card.source_line, card.raw_text);
    std::size_t bytes = evidence.block.raw_text.size();
    for (const auto& prior : data.sources) {
        Require(prior.block.keyword != "*CONTROL_HOURGLASS" &&
                bytes <= limits.metadata_bytes && prior.block.raw_text.size() <= limits.metadata_bytes - bytes,
                "Repeated or excessive global hourglass evidence");
        bytes += prior.block.raw_text.size();
    }
    Require(bytes <= limits.metadata_bytes && data.sources.size() < limits.blocks,
            "Airbag source evidence exceeds capacity");
    data.sources.push_back(std::move(evidence));
}
} // namespace crash::modelio::solid_source::detail
