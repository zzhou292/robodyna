#include "Internal.h"
#include "modelio/source_assembly/JsonReader.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include <cmath>
namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail {
namespace {
bool Starts(const std::string& text, const char* prefix) { return text.rfind(prefix, 0) == 0; }
}
void CheckContext(const c::CorrectedNodalSource& corrected, const modelio::self_contact::OriginalSelection& selection) {
    namespace r = modelio::assembly::reader;
    const auto& physical = corrected.pre_correction().physical();
    const auto& canonical = coated::detail::CheckSource(physical, selection, coated::ConfigFor(physical), {});
    const auto& source = corrected.pre_correction().provenance();
    Require(&canonical == &physical.shell_source().references().source().canonical().data(),
        "Main coefficient handles do not share canonical source authority");
    Require(!corrected.provenance().source_digest.empty() &&
        corrected.provenance().source_digest == source.import_source_digest &&
        corrected.provenance().interfaces.disposition == c::InterfaceDisposition::CompleteNoApplicableType24,
        "Main coefficient complete closed import/control context differs");
    Require(tl::math::SameScalarBits(source.units.length_m, canonical.inputs.units.length_to_m) &&
        tl::math::SameScalarBits(source.units.mass_kg, canonical.inputs.units.mass_to_kg) &&
        tl::math::SameScalarBits(source.units.time_s, canonical.inputs.units.time_to_s),
        "Main coefficient native material and coordinate units differ");
    output::Document document;
    document.Parse(canonical.canonical_bytes.data(), canonical.canonical_bytes.size());
    Require(!document.HasParseError() && document.IsObject(),"Main coefficient canonical source inventory is invalid");
    const auto& files = r::Member(document,"source_files");
    Require(files.IsObject(),"Main coefficient complete source files missing");
    for (const auto& file:files.GetObject()) {
        const auto& counts = r::Member(file.value,"keyword_counts");
        Require(counts.IsObject(),"Main coefficient complete keyword census missing");
        for (const auto& item:counts.GetObject()) {
            const std::string keyword(item.name.GetString(), item.name.GetStringLength());
            (void)r::Unsigned(item.value);
            if ((Starts(keyword,"*PART") && keyword != "*PART") ||
                (Starts(keyword,"*ELEMENT_SHELL") && keyword != "*ELEMENT_SHELL") ||
                Starts(keyword,"*CONTROL_ADAPT") || Starts(keyword,"*DEFINE_ADAPT") ||
                Starts(keyword,"*INCLUDE_RADIOSS") || Starts(keyword,"*INCLUDE_LS") ||
                (Starts(keyword,"*INITIAL_") && keyword != "*INITIAL_VELOCITY_GENERATION") ||
                keyword.find("DRAPE") != std::string::npos || keyword.find("PERTURB") != std::string::npos ||
                keyword.find("XFEM") != std::string::npos || keyword.find("AMS") != std::string::npos)
                Reject(Status::UnsupportedSource,"Main coefficient source phase/grouping control is outside the closed profile");
        }
    }
    // The source-owned CorrectedNodalSource only admits the qualified fresh
    // direct-key PO import profile. Its complete census, plus the checks above,
    // selects native no-Ioffset/no-AMS/no-adaptivity/no-INIBRI-FILL rules.
    // Mechanical NLOC/IPOS remains in the physical declarations; it is not an
    // optional X_C projection. No caller boolean selects this authority.
    const auto& contact = selection.data().sources.at(0);
    Require(contact.block.keyword == "*CONTACT_AUTOMATIC_SINGLE_SURFACE" && contact.cards.size() == 8 &&
        modelio::assembly::reader::auxiliary::Trim(contact.cards[2].second).empty(),
        "Main coefficient selected default SFS and gap-control card differs");
}
} // namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail
