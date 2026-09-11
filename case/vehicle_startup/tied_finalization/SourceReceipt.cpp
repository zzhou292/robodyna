#include "Internal.h"
#include "modelio/tied_shell/packing/Internal.h"
#include "modelio/tied_shell/Internal.h"
#include <map>
namespace crash::cases::vehicle_startup::tied_finalization_detail {
namespace {
using namespace modelio::assembly::reader;
bool Contact(const std::string& keyword) {
    return keyword.rfind("*CONTACT_", 0) == 0 || keyword.rfind("/INTER/", 0) == 0;
}
void CheckContactBlock(const tied::SourceEvidence& source, const Value& row, const std::string& filename) {
    const auto& block = source.block;
    output::Require(filename == block.filename && Text(row, "file") == filename &&
            Text(row, "keyword") == block.keyword && Unsigned(row, "first_line") == block.first_line &&
            Unsigned(row, "last_line") == block.last_line && Text(row, "source_block_sha256") == block.sha256,
            "Tied finalization contact source association changed");
}
}
TiedFinalizationReceipt Receipt(const tied::source::CanonicalData& canonical, const tied::Data& declaration,
        const tied::PackingData& packing, TiedFinalizationLimits limits) {
    using namespace modelio::assembly::reader;
    using output::Require;
    Require(canonical.canonical_bytes.size() <= limits.metadata_bytes, "Tied finalization metadata cap exceeded");
    tied::packing_detail::CheckPolicy(declaration);
    tied::detail::CheckContact(declaration.sources.at(declaration.contact_source),
                               declaration.slave_set_id, declaration.master_set_id);
    Require(packing.master_rows.size() == declaration.masters.size() &&
            packing.master_ranks.size() == declaration.masters.size(), "Tied finalization packing extent changed");
    output::Document doc;
    doc.Parse(canonical.canonical_bytes.data(), canonical.canonical_bytes.size());
    Require(!doc.HasParseError() && doc.IsObject(), "Invalid tied finalization source metadata");
    const auto& files = Member(doc, "source_files");
    Require(files.IsObject() && files.MemberCount() && files.MemberCount() <= limits.source_files,
            "Tied finalization source file cap exceeded");
    TiedFinalizationReceipt receipt;
    receipt.contact_source = declaration.contact_source;
    receipt.source_file_count = files.MemberCount();
    std::size_t total_blocks = 0;
    for (const auto& file : files.GetObject()) {
        const std::string filename(file.name.GetString(), file.name.GetStringLength());
        const auto& blocks = Array(file.value, "blocks", limits.source_blocks);
        Require(blocks.Size() <= limits.source_blocks-total_blocks, "Tied finalization source block cap exceeded");
        total_blocks += blocks.Size();
        const auto& counts = Member(file.value, "keyword_counts");
        Require(counts.IsObject(), "Missing tied finalization contact census");
        std::map<std::string, std::size_t> actual;
        std::size_t previous = 0;
        for (const auto& row : blocks.GetArray()) {
            const auto first = Unsigned(row, "first_line"), last = Unsigned(row, "last_line");
            Require(Text(row, "file") == filename && first > previous && last >= first,
                    "Tied finalization source block order changed");
            previous = last;
            const auto keyword = Text(row, "keyword");
            if (!Contact(keyword)) continue;
            ++actual[keyword];
            ++receipt.source_contact_count;
            if (keyword == "*CONTACT_TIED_SHELL_EDGE_TO_SURFACE") {
                CheckContactBlock(declaration.sources.at(declaration.contact_source), row, filename);
                ++receipt.type2_count;
            } else {
                // Exact original non-TYPE2 converter roles. Any new contact
                // kind requires its own declared conversion proof.
                Require(keyword == "*CONTACT_AUTOMATIC_SINGLE_SURFACE" || keyword == "*CONTACT_INTERIOR",
                        "Unresolved additional native interface role");
            }
        }
        for (const auto& count : counts.GetObject()) {
            const std::string keyword(count.name.GetString(), count.name.GetStringLength());
            if (Contact(keyword))
                Require(Unsigned(count.value, limits.source_blocks) == actual[keyword],
                        "Tied finalization contact census omitted a block");
        }
        for (const auto& [keyword, count] : actual)
            Require(counts.HasMember(keyword.c_str()) && Unsigned(counts, keyword.c_str()) == count,
                    "Tied finalization contact census omitted a keyword");
    }
    Require(receipt.type2_count == 1, "Tied finalization requires exactly one original TYPE2 interface");
    tied::SourceId previous = 0;
    for (const auto& node : declaration.slave_nodes) {
        Require(node.id > previous, "Tied finalization requires unique ascending original NSV identities");
        previous = node.id;
    }
    Require(!declaration.slave_nodes.empty(), "Empty tied finalization source population");
    // Qualified additive PART-clause INPOINT import is ordinary IS1=2.
    // Fresh serial LECINT increments each unique NSV once; none reaches two.
    receipt.unique_original_slaves = declaration.slave_nodes.size();
    receipt.type2_ordinal = 1;
    receipt.is1 = 2;
    receipt.level = 28;
    receipt.ignore = 2;
    receipt.projection = 1;
    return receipt;
}
}
