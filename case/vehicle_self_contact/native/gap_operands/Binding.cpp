#include "Internal.h"
#include "modelio/source_assembly/JsonReader.h"
#include "lib_src/collision/self_contact_filters/Environment.h"
#include <cmath>
namespace crash::cases::vehicle_self_contact::native::gap_operands::detail {
namespace r=modelio::assembly::reader;
namespace { bool Starts(const std::string& a,const char* b) { return a.rfind(b,0)==0; } }
std::size_t CheckKeywordInventory(const output::Value& files) {
    Require(files.IsObject(),"Gap source complete keyword inventory unavailable");
    std::size_t checked=0;
    for(const auto& file:files.GetObject()) {
        const auto& counts=r::Member(file.value,"keyword_counts");
        Require(counts.IsObject(),"Gap source member keyword inventory unavailable");
        for(const auto& entry:counts.GetObject()) {
            const auto count=r::Unsigned(entry.value);
            if(!count)continue;
            const std::string keyword(entry.name.GetString(),entry.name.GetStringLength());
            if((Starts(keyword,"*PART")&&keyword!="*PART") ||
                (Starts(keyword,"*ELEMENT_SHELL")&&keyword!="*ELEMENT_SHELL") ||
                Starts(keyword,"*INITIAL_SHELL") || Starts(keyword,"*INITIAL_THICKNESS") ||
                keyword.find("DRAPE")!=std::string::npos || Starts(keyword,"*INCLUDE_RADIOSS") ||
                Starts(keyword,"*INCLUDE_LS") || Starts(keyword,"*CONTROL_ADAPT") ||
                Starts(keyword,"*DEFINE_ADAPT"))
                Reject(Status::UnsupportedSource,"Unproved gap thickness/default source writer");
            Require(count<=SIZE_MAX-checked,"Gap source keyword census overflow");
            checked+=count;
        }
    }
    return checked;
}
SourceProof Context(const source::CorrectedNodalSource& corrected) {
    const auto& seed=corrected.pre_correction();
    const auto& physical=seed.physical();
    const auto& canonical=physical.shell_source().references().source().canonical().data();
    Require(corrected.provenance().source_digest==seed.provenance().import_source_digest &&
        !corrected.provenance().source_digest.empty() &&
        corrected.provenance().interfaces.disposition==source::InterfaceDisposition::CompleteNoApplicableType24,
        "Gap operands lack the same complete fresh direct import authority");
    Require(physical.source_domain().policy()==modelio::physical_domain::Policy::RetainedShellAssembliesVehicleSupportsV5,
        "Gap operands require the complete named V5 physical domain");
    Require(tlfea::contact::self_contact_filters::CompatibleHostArithmetic(),
        "Gap source requires round-to-nearest, gradual underflow and masked traps");
    const auto units=seed.provenance().units;
    Require(SameUnits(units,{canonical.inputs.units.length_to_m,canonical.inputs.units.mass_to_kg,
        canonical.inputs.units.time_to_s}),"Gap source native units differ from canonical authority");
    output::Document document;
    document.Parse(canonical.canonical_bytes.data(),canonical.canonical_bytes.size());
    Require(!document.HasParseError()&&document.IsObject(),"Invalid closed gap source metadata");
    SourceProof proof;
    proof.checked_keywords=CheckKeywordInventory(r::Member(document,"source_files"));
    // CorrectedNodalSource privately retains the qualified fresh-direct import
    // and default weld/regular-joint generators. Original plain PART and these
    // generators have no THICK assignment: this is a universal absence proof,
    // not a lookup of unknown generated PART IDs followed by a zero fallback.
    return proof;
}
void CheckMaximumTerm(double value) {
    if(!std::isfinite(value)||value<0||std::signbit(value))
        Reject(Status::NeedsSourceOrder,"Gap MAX certificate requires finite nonnegative terms with only positive zero");
}
}
