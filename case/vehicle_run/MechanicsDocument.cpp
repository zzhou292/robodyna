#include "MechanicsDocument.h"

namespace crash::cases::vehicle_run::detail {
namespace {
using namespace output;
void Object(Document& document, const char* name, Document child) {
    Value value;
    value.CopyFrom(child, document.GetAllocator());
    document.AddMember(rapidjson::StringRef(name), value, document.GetAllocator());
}
Document Work(const WorkObservation& work) {
    Document document;
    document.SetObject();
    Number(document, "last_increment_j", work.last_increment_j);
    Number(document, "accepted_increment_sum_j", work.accepted_increment_sum_j);
    Number(document, "peak_absolute_increment_j", work.peak_absolute_increment_j);
    return document;
}
Document Plastic(const PlasticWorkObservation& plastic) {
    auto document = Work(plastic.work);
    Boolean(document, "positive_increment_observed", plastic.first_positive_epoch != 0);
    if (plastic.first_positive_epoch) {
        Integer(document, "first_positive_epoch", plastic.first_positive_epoch);
        Number(document, "first_positive_time_s", plastic.first_positive_time_s);
    }
    return document;
}
Document NativeStep(const NativeStepObservation& step) {
    Document document;
    document.SetObject();
    Number(document, "last_s", step.last_s);
    Number(document, "minimum_observed_s", step.minimum_observed_s);
    return document;
}
Document Maximum(const MotionMaximum& maximum) {
    Document document;
    document.SetObject();
    Number(document, "last", maximum.last);
    Number(document, "peak", maximum.peak);
    return document;
}
Document Solids(const SolidMechanicsTotals& solids) {
    Document document;
    document.SetObject();
    constexpr const char* names[]{"solid18_law36", "solid24_law42", "solid6z_law42",
        "solid18_law44", "solid18_law90"};
    Value families(rapidjson::kArrayType);
    for (std::size_t family = 0; family < solids.parents.size(); ++family) {
        Document row;
        row.SetObject();
        String(row, "family", names[family]);
        Integer(row, "parents", solids.parents[family]);
        Boolean(row, "available", solids.parents[family] != 0);
        if (solids.parents[family]) {
            Object(row, "native_work", Work(solids.native_work[family]));
            Object(row, "included_hourglass_work", Work(solids.hourglass_work[family]));
        }
        Value value;
        value.CopyFrom(row, document.GetAllocator());
        families.PushBack(value, document.GetAllocator());
    }
    document.AddMember("families", families, document.GetAllocator());
    Object(document, "included_law36_law44_plastic_work", Plastic(solids.metal_plastic_work));
    Object(document, "accepted_rhs_kick_work", Work(solids.rhs_kick_work));
    Object(document, "accepted_rhs_drift_work", Work(solids.rhs_drift_work));
    Object(document, "native_element_dt", NativeStep(solids.native_step));
    return document;
}
Document Beam(const BeamMechanicsTotals& beam) {
    Document document;
    document.SetObject();
    Integer(document, "parents", beam.parents);
    Object(document, "native_membrane_shear_work", Work(beam.native_work[0]));
    Object(document, "native_flexural_torsional_work", Work(beam.native_work[1]));
    Object(document, "included_plastic_work", Plastic(beam.plastic_work));
    Object(document, "accepted_rhs_kick_work", Work(beam.rhs_kick_work));
    Object(document, "accepted_rhs_drift_work", Work(beam.rhs_drift_work));
    Object(document, "native_element_dt", NativeStep(beam.native_step));
    return document;
}
Document Motion(const MotionTotals& motion) {
    Document document;
    document.SetObject();
    Integer(document, "physical_nodes", motion.nodes);
    String(document, "reference", "original uniform translation; component maxima, not strain or fitted rigid-motion residual");
    Object(document, "translation_departure_m", Maximum(motion.translation_departure_m));
    Object(document, "velocity_departure_m_s", Maximum(motion.velocity_departure_m_s));
    Object(document, "orientation_component_departure", Maximum(motion.orientation_component_departure));
    Object(document, "spin_component_rad_s", Maximum(motion.spin_component_rad_s));
    return document;
}
} // namespace
output::Document MechanicsDocument(const MechanicsTotals& totals) {
    Document document;
    document.SetObject();
    String(document, "schema", "robo_dyna.accepted_mechanics_summary.v1");
    Boolean(document, "available", totals.available);
    if (!totals.available) return document;
    String(document, "scope", "accepted diagnostic increments only; excludes TIME0 energy, kinetic and total energy");
    String(document, "work_accounting", "hourglass and plastic work are included components of native work; do not add again");
    String(document, "plasticity_scope", "first positive reported plastic-work increment; point plastic strain and residual deformation unavailable");
    String(document, "native_dt_scope", "unscaled element diagnostics, not the admitted post-CIN structural or contact step limit");
    Integer(document, "accepted_intervals", totals.intervals);
    Integer(document, "owner_id", totals.owner_id);
    Integer(document, "source_instance_id", totals.source_instance_id);
    Integer(document, "configuration_id", totals.configuration_id);
    Integer(document, "qualification_id", totals.qualification_id);
    Integer(document, "last_attempt", totals.last_attempt);
    Number(document, "last_time_s", totals.last_time_s);
    Number(document, "last_velocity_time_s", totals.last_velocity_time_s);
    Number(document, "fixed_dt_s", totals.fixed_dt_s);
    Boolean(document, "has_type45", totals.has_type45);
    Boolean(document, "has_beam18", totals.has_beam18);
    Object(document, "solids", Solids(totals.solids));
    if (totals.has_beam18) Object(document, "beam18", Beam(totals.beam18));
    Object(document, "motion", Motion(totals.motion));
    return document;
}
} // namespace crash::cases::vehicle_run::detail
