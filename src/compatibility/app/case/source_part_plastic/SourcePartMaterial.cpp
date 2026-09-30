#include "SourcePartMaterial.h"
#include "output/ArtifactIO.h"
#include "qualification/source_contact/SourcePartContactFixture.h"
#include <cmath>
#include <limits>
#include <string_view>

namespace crash::cases::source_part_plastic {
namespace {
namespace io = crash::output;
namespace source = crash::qualification::source_contact;
const io::Value& Member(const io::Value& object, const char* key) {
    io::Require(object.IsObject() && object.HasMember(key), "Missing source material member");
    return object[key];
}
const io::Value& Array(const io::Value& object, const char* key, std::size_t count) {
    const auto& value = Member(object, key);
    io::Require(value.IsArray() && value.Size() == count, "Invalid source material array shape");
    return value;
}
std::uint64_t Unsigned(const io::Value& object, const char* key, std::uint64_t cap = UINT64_MAX) {
    const auto& value = Member(object, key);
    io::Require(value.IsUint64() && value.GetUint64() <= cap, "Invalid source material integer");
    return value.GetUint64();
}
double Real(const io::Value& value) {
    io::Require(value.IsNumber() && std::isfinite(value.GetDouble()), "Nonfinite source material scalar");
    return value.GetDouble();
}
double Real(const io::Value& object, const char* key) { return Real(Member(object, key)); }
bool TextIs(const io::Value& object, const char* key, std::string_view text) {
    const auto& value = Member(object, key);
    return value.IsString() && std::string_view(value.GetString(), value.GetStringLength()) == text;
}
void Flag(const io::Value& object, const char* key, bool expected) {
    const auto& value = Member(object, key);
    io::Require(value.IsBool() && value.GetBool() == expected, "Unexpected source material scope");
}
void Same(double a, double b) {
    io::Require(io::Bits(a) == io::Bits(b), "Source material declaration association changed");
}
} // namespace

MaterialReport LoadPinnedSourcePartMaterial(const std::filesystem::path& path, SourcePartMaterial* output) {
    if (!output) return {MaterialStatus::InvalidArgument, "Null source material output"};
    std::string bytes;
    try { bytes = io::ReadBounded(path, source::ReadinessBytes); }
    catch (const std::exception& error) { return {MaterialStatus::ReadFailure, error.what()}; }
    try {
        if (bytes.size() != source::ReadinessBytes || io::Sha256(bytes) != source::ReadinessSha256)
            return {MaterialStatus::HashMismatch, "Source material requires the unchanged pinned readiness artifact"};
        io::Document document;
        document.Parse<rapidjson::kParseFullPrecisionFlag | rapidjson::kParseIterativeFlag>(bytes.data(), bytes.size());
        io::Require(!document.HasParseError() && document.IsObject() &&
            TextIs(document, "schema", "robo-dyna.source-part-readiness.v1"), "Invalid source readiness JSON");
        const auto& declaration = Member(document, "source_declarations");
        io::Require(TextIs(declaration, "schema", "robo-dyna.source-part-declarations.v1"),
            "Unexpected source declaration schema");
        Flag(declaration, "source_declarations_validated", true);
        Flag(declaration, "simulation_ready", false);
        const auto& data = Member(declaration, "declarations");
        const auto& part = Member(data, "part");
        const auto& material = Member(data, "material");
        const auto& section = Member(data, "section");
        const auto& curve = Member(data, "hardening_curve");
        SourcePartMaterial candidate;
        auto& d = candidate.declaration_;
        d.part_id = Unsigned(part, "part_id");
        d.material_id = Unsigned(material, "material_id");
        d.section_id = Unsigned(section, "section_id");
        d.curve_id = Unsigned(curve, "curve_id");
        io::Require(d.part_id == source::PartId && d.material_id == d.part_id && d.section_id == d.part_id &&
            Unsigned(part, "material_id") == d.material_id && Unsigned(part, "section_id") == d.section_id &&
            d.curve_id == 2100270 && Unsigned(material, "hardening_curve_id") == d.curve_id,
            "Source material/section/curve identity mismatch");
        d.young_pa = Real(material, "young_pa");
        d.poisson_ratio = Real(material, "poisson_ratio");
        d.density_kg_m3 = Real(material, "density_kg_m3");
        d.supplied_sigy_pa = Real(material, "supplied_sigy_pa");
        d.source_rate_coefficient_per_s = Real(material, "rate_coefficient_per_s");
        d.source_rate_exponent = Real(material, "rate_exponent");
        d.source_vp = static_cast<int>(Unsigned(material, "rate_type", 2));
        d.source_elform = static_cast<unsigned>(Unsigned(section, "source_elform", 16));
        d.through_thickness_points = static_cast<unsigned>(Unsigned(section, "through_thickness_points", 11));
        const auto& thickness = Array(section, "thickness_m", 4);
        d.thickness_m = Real(thickness[0]);
        for (const auto& value : thickness.GetArray()) Same(Real(value), d.thickness_m);
        io::Require(d.young_pa == 200.e9 && d.poisson_ratio == .3 && d.density_kg_m3 > 0 &&
            d.thickness_m == .001648 && d.supplied_sigy_pa == 270.e6 &&
            d.source_rate_coefficient_per_s == 8000 && d.source_rate_exponent == 8 && d.source_vp == 0 &&
            d.source_elform == 2 && d.through_thickness_points == 3,
            "Unexpected original source material/section tuple");
        const auto& audit = Member(document, "surface_mass_audit");
        Same(d.density_kg_m3, Real(audit, "density_kg_m3"));
        Same(d.thickness_m, Real(audit, "thickness_m"));
        const auto& cards = Array(material, "cards", 4);
        for (unsigned i = 0; i < 4; ++i) {
            d.material_blank_masks[i] = static_cast<std::uint16_t>(Unsigned(cards[i], "blank_field_mask", 255));
            d.material_source_lines[i] = static_cast<std::uint32_t>(Unsigned(cards[i], "source_line", UINT32_MAX));
            const auto& values = Array(cards[i], "values", 8);
            for (unsigned field = 0; field < 8; ++field)
                io::Require(values[field].IsNull() == bool(d.material_blank_masks[i] & (1u << field)),
                    "Source material blank mask/value mismatch");
        }
        const auto& first = Array(cards[0], "values", 8);
        const auto& second = Array(cards[1], "values", 8);
        Same(Real(second[0]), d.source_rate_coefficient_per_s);
        Same(Real(second[1]), d.source_rate_exponent);
        io::Require(Real(second[2]) == double(d.curve_id) && Real(second[4]) == d.source_vp,
            "Source rate card association mismatch");
        d.source_lcsr_blank = second[3].IsNull();
        d.source_failure_blank = first[6].IsNull();
        d.source_tdel_blank = first[7].IsNull();
        io::Require(d.source_lcsr_blank && d.source_failure_blank && d.source_tdel_blank &&
            Member(material, "supplied_etan_pa").IsNull(), "Unsupported active source material option");
        // Pinned direct-import chain: CPP_LSD2RAD_CONVERTOR passes the converted
        // ModelViewPO directly to the solver; absent Fcut -> unavailable getter
        // -> explicit zero in CPP_GET_FLOATV_FLOATD -> HM_READ_MAT44 default.
        // This is distinct from exporting/re-reading a Radioss CFG default.
        d.filter_cutoff = FilterCutoffResolution::OpenRadiossDirectImportDefault;
        d.resolved_filter_cutoff_per_s = 10000.;
        const auto& strain = Array(curve, "plastic_strain", MaterialCurvePoints);
        const auto& stress = Array(curve, "stress_pa", MaterialCurvePoints);
        for (unsigned i = 0; i < MaterialCurvePoints; ++i) {
            candidate.plastic_strain_[i] = Real(strain[i]);
            candidate.yield_stress_pa_[i] = Real(stress[i]);
        }
        io::Require(candidate.plastic_strain_.front() == 0 && candidate.plastic_strain_.back() == .3 &&
            candidate.yield_stress_pa_.front() == 270.e6 && candidate.yield_stress_pa_.back() == 362.e6,
            "Unexpected original source curve endpoints");
        tl::material::TabulatedShellPlasticityParameters checked;
        io::Require(tl::material::PrepareTabulatedShellPlasticity(d.young_pa, d.poisson_ratio, d.density_kg_m3,
            {candidate.plastic_strain_.data(), candidate.yield_stress_pa_.data(), MaterialCurvePoints}, checked) ==
            tl::material::TabulatedShellPlasticityStatus::Ok, "Source curve exceeds the admitted point-parameter domain");
        candidate.prepared_ = true;
        *output = candidate;
        return {MaterialStatus::Ok, "Authenticated source material and actual rate declarations copied; solver policy remains explicit"};
    } catch (const std::exception& error) { return {MaterialStatus::InvalidDeclaration, error.what()}; }
}
} // namespace crash::cases::source_part_plastic
