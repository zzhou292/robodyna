#include "FrameGeometryData.h"

namespace crash::visual::full_shell::detail {
namespace {
output::ReplayScalarApplicability Applicability(output::full_shell::PlasticField field) {
    using F = output::full_shell::PlasticField;
    using A = output::ReplayScalarApplicability;
    switch (field) {
        case F::NativeEquivalentPlasticStrain: return A::NativeValue;
        case F::NotApplicable: return A::NotApplicable;
        case F::Unavailable: return A::Unavailable;
    }
    throw std::runtime_error("Unknown source plastic-field applicability");
}
} // namespace
std::vector<output::ReplayParentScalar> InitialFields(const output::full_shell::Context& context) {
    std::vector<output::ReplayParentScalar> fields;
    fields.reserve(context.parents().size());
    for (const auto& parent : context.parents())
        fields.push_back({parent.source_element, 0, Applicability(parent.plastic)});
    return fields; // Private storage markers, never exposed as an initial frame.
}
void StageFields(const output::full_shell::Context& context, const output::full_shell::FrameRecord& frame,
        std::vector<output::ReplayParentScalar>& fields) {
    const auto maxima = output::full_shell::ParentPlasticMaxima(context, frame);
    output::Require(fields.size() == maxima.size(), "Incomplete staged parent field extent");
    for (std::size_t i = 0; i < fields.size(); ++i) {
        const bool native = fields[i].applicability == output::ReplayScalarApplicability::NativeValue;
        output::Require(native == maxima[i].has_value(), "Native point applicability differs from frame layout");
    }
    for (std::size_t i = 0; i < fields.size(); ++i) {
        if (maxima[i]) fields[i].value = *maxima[i];
        else fields[i].value = 0; // Uninterpreted marker, selected only through its non-native tag.
    }
}
} // namespace crash::visual::full_shell::detail
