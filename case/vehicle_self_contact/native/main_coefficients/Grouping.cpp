#include "Internal.h"
#include "modelio/tied_shell/Internal.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail {
namespace r = modelio::assembly::reader;
namespace tied = modelio::tied_shell;
namespace {
bool Starts(const std::string& value, const char* prefix) { return value.rfind(prefix, 0) == 0; }
}
bool OrdinaryPartControls(const output::Value& part) {
    const auto& fields = r::Array(part, "raw_fields", 8, 8);
    if (r::Unsigned(part, "blank_field_mask") != 248) return false;
    for (unsigned k = 3; k < 8; ++k) if (r::Real(fields[k]) != 0.) return false;
    return true;
}
bool GroupingKeywords(const output::Value& files) {
    Require(files.IsObject(), "Grouping source inventory is not an object");
    for (const auto& file : files.GetObject()) {
        const auto& counts = r::Member(file.value, "keyword_counts");
        Require(counts.IsObject(), "Grouping source keyword counts unavailable");
        for (const auto& item : counts.GetObject()) {
            if (!r::Unsigned(item.value)) continue;
            const std::string keyword(item.name.GetString(), item.name.GetStringLength());
            // These independent modifiers can change the pre-MID sorting key
            // despite identical material/property cards. They do not invalidate
            // an already order-independent coefficient value.
            if (Starts(keyword, "*MAT_ADD") || keyword.find("THERM") != std::string::npos ||
                keyword.find("EXPANSION") != std::string::npos || keyword.find("XFEM") != std::string::npos ||
                keyword.find("ADAPT") != std::string::npos || keyword.find("AMS") != std::string::npos ||
                Starts(keyword, "*PART_") || Starts(keyword, "*INCLUDE_RADIOSS") ||
                Starts(keyword, "*INCLUDE_LS") || Starts(keyword, "*DEFINE_ELEMENT_DEATH") ||
                Starts(keyword, "*DEFINE_ELEMENT_BIRTH") ||
                (Starts(keyword, "*DEFINE_") && keyword != "*DEFINE_CURVE" &&
                    keyword != "*DEFINE_COORDINATE_NODES" && keyword != "*DEFINE_TRANSFORMATION") ||
                (Starts(keyword, "*CONTROL_") && keyword != "*CONTROL_ACCURACY" && keyword != "*CONTROL_CONTACT" &&
                    keyword != "*CONTROL_CPU" && keyword != "*CONTROL_ENERGY" && keyword != "*CONTROL_HOURGLASS" &&
                    keyword != "*CONTROL_OUTPUT" && keyword != "*CONTROL_SHELL" && keyword != "*CONTROL_SOLID" &&
                    keyword != "*CONTROL_TERMINATION" && keyword != "*CONTROL_TIMESTEP")) return false;
        }
    }
    return true;
}
bool GroupingContext(const modelio::self_contact::OriginalSelection& selected, const std::string& member) {
    const auto& selection = selected.data();
    Require(member.size() <= modelio::self_contact::Limits{}.combine_member_bytes &&
        output::Sha256(member) == selection.combine_sha256, "Grouping control member hash or extent differs");
    output::Document document;
    const auto& bytes = selected.canonical().data().canonical_bytes;
    document.Parse(bytes.data(), bytes.size());
    Require(!document.HasParseError() && document.IsObject(), "Grouping canonical inventory unavailable");
    const auto& files = r::Member(document, "source_files");
    if (!GroupingKeywords(files)) return false;
    tied::detail::Requests requested;
    for (const auto& file : files.GetObject()) {
        const std::string filename(file.name.GetString(), file.name.GetStringLength());
        for (const auto& block : r::Array(file.value, "blocks", 8192)) {
            if (r::Text(block, "keyword") != "*CONTROL_TIMESTEP") continue;
            if (filename != selection.combine_filename) return false;
            tied::detail::Request request;
            request.evidence.block = {filename, "*CONTROL_TIMESTEP", {}, r::Text(block, "source_block_sha256"),
                r::Unsigned(block, "first_line"), r::Unsigned(block, "last_line")};
            Require(requested.emplace(request.evidence.block.first_line, std::move(request)).second,
                "Duplicate native timestep control block");
        }
    }
    if (requested.size() != 1) return false;
    auto limits = tied::Limits{};
    limits.member_bytes = modelio::self_contact::Limits{}.combine_member_bytes;
    const auto evidence = tied::detail::ReadRequestedSources(requested, member, limits);
    return NoOptionalAmsCard(evidence);
}
bool NoOptionalAmsCard(const std::vector<modelio::tied_shell::SourceEvidence>& evidence) {
    Require(evidence.size() == 1, "Native timestep source extraction changed");
    auto cards = evidence.front().cards;
    while (!cards.empty() && r::auxiliary::Trim(cards.back().second).empty()) cards.pop_back();
    // No optional card means no IMSCL definition. The pinned converter creates
    // /AMS only for IMSCLOptFlag1/2/3. HM_READ_SMS explicitly starts ISMS=0 and
    // sets it only while reading /AMS; hence native key2 JSMS=0 for every part.
    // This certificate does not modify DT2MS, the timestep, or physical masses.
    return cards.size() == 1;
}
}
