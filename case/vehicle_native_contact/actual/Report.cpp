#include "Report.h"
#include <cstdlib>
namespace crash::cases::vehicle_native_contact::test {
double Seconds(Clock::time_point start) { return std::chrono::duration<double>(Clock::now() - start).count(); }
std::filesystem::path Destination() {
    const char* text = std::getenv("ROBO_NATIVE_VEHICLE_CASE_OUTPUT");
    output::Require(text && *text, "Missing create-only native vehicle case output");
    const std::filesystem::path path(text);
    output::Require(std::filesystem::create_directory(path), "Native vehicle case output already exists");
    return path;
}
output::Document Document(const char* stage) {
    output::Document doc;
    doc.SetObject();
    output::String(doc, "schema", "robo_dyna.native_vehicle_case_qualification.v1");
    output::String(doc, "stage", stage);
    output::Boolean(doc, "physical_owner_created", false);
    return doc;
}
void Sources(output::Document& doc, const detail::SourceInputs& source) {
    output::String(doc, "self_source_digest", source.self.provenance().output_digest);
    output::String(doc, "wall_source_digest", source.wall.provenance().output_digest);
    output::String(doc, "controls_digest", source.controls.provenance().output_digest);
    output::Integer(doc, "physical_nodes", source.owner.physical().domain()->node_count());
    const auto& solids=source.owner.execution_source().mechanical().solids();
    if(const auto* controls=solids.control_selection();controls&&
        controls->profile()==tl::fea::solids::control::Profile::SourceDeclared) {
        output::Integer(doc,"solid_source_control_count",controls->controlled_count());
        output::Integer(doc,"solid_model_owned_payload_bytes",solids.owned_payload_bytes());
        std::size_t curve_points=0;
        for(const auto& material:solids.materials36())curve_points+=material.value.curve.count;
        for(const auto& material:solids.materials44())curve_points+=material.value.curve.count;
        for(const auto& material:solids.materials90())curve_points+=material.value.curve().count;
        output::Integer(doc,"solid_owned_curve_points",curve_points);
        output::Integer(doc,"solid_native_packet_count",controls->packets().size());
        output::Integer(doc,"solid_native_nvsiz",controls->native_nvsiz());
        output::Integer(doc,"solid_compiled_mvsiz",controls->compiled_mvsiz());
        output::Integer(doc,"solid18_count",solids.solid18().size());
        output::Integer(doc,"solid24_count",solids.solid24().size());
        output::Integer(doc,"solid6z_count",solids.solid6z().size());
        output::Integer(doc,"solid18_law44_count",solids.solid18_law44().size());
        output::Integer(doc,"solid18_law90_count",solids.solid18_law90().size());
    }
    output::Integer(doc, "self_primaries", source.self.snapshot().primary_count);
    output::Integer(doc, "self_mains", source.self.snapshot().main_count);
    output::Integer(doc, "self_secondaries", source.self.main_source().secondary_nodes().size());
    output::Integer(doc, "wall_secondaries", source.wall.secondary_nodes().size());
    output::Value table(rapidjson::kArrayType);
    for (const auto& row : source.controls.interfaces()) {
        output::Document one;
        one.SetObject();
        output::Integer(one, "kind", static_cast<unsigned>(row.kind));
        output::Integer(one, "native_id", row.native_id);
        output::Integer(one, "native_storage_ordinal", row.native_storage_ordinal);
        output::String(one, "source_file", row.source.filename);
        output::Integer(one, "source_first_line", row.source.first_line);
        table.PushBack(output::Value(one, doc.GetAllocator()), doc.GetAllocator());
    }
    doc.AddMember("interface_table", table, doc.GetAllocator());
}
void Plan(output::Document& doc, const Forecast& f, bool complete) {
    output::Boolean(doc, "complete_runtime_forecast", complete);
    output::Integer(doc, "source_construction_peak_bytes", f.sources.construction_peak);
    output::Integer(doc, "source_retained_bytes", f.sources.retained_bytes);
    output::Integer(doc, "case_packing_retained_bytes", f.packing_retained);
    output::Integer(doc, "host_preparation_ceiling_bytes", f.host_preparation_ceiling);
    output::Integer(doc, "qualification_export_reservation_bytes", ExportBytes);
    output::Integer(doc, "qualification_host_guard_bytes", GuardBytes);
    if (!complete) return;
    output::Integer(doc, "prepared_sources_retained_bytes", f.prepared_source_retained);
    output::Integer(doc, "complete_retained_host_bytes", f.retained_host_bytes);
    output::Integer(doc, "complete_peak_host_bytes", f.peak_host_bytes);
    output::Integer(doc, "steady_device_bytes", f.steady_device_bytes);
    output::Integer(doc, "complete_peak_device_bytes", f.peak_device_bytes);
    output::Integer(doc, "census_conservative_peak_host_bytes", f.census_peak_host_bytes);
    output::Integer(doc, "census_peak_device_bytes", f.census_peak_device_bytes);
    output::Boolean(doc, "fits_runtime_limits", f.fits_runtime_limits);
    output::Integer(doc,"admitted_solid_packet_blocks",f.physical.startup.solid_packet_blocks);
    output::Integer(doc,"admitted_solid_worker_slots",f.physical.startup.solid_worker_slots);
    output::Value table(rapidjson::kArrayType);
    for (const auto& entry : f.sources.interfaces) {
        const auto index = entry.role == Role::Self ? 0u : 1u;
        const auto& plan = f.contact[index];
        output::Document one;
        one.SetObject();
        output::Integer(one, "native_id", entry.native_id);
        output::String(one, "role", entry.role == Role::Self ? "self" : "mesh_wall");
        output::Integer(one, "runtime_device_bytes", plan.transaction.device_bytes);
        output::Integer(one, "inventory_device_bytes", plan.transaction.inventory_device_bytes);
        output::Integer(one, "maintenance_device_bytes", plan.transaction.maintenance_device_bytes);
        output::Integer(one, "incidence_device_bytes", plan.transaction.incidence_device_bytes);
        output::Integer(one, "normal_device_bytes", plan.transaction.normal_device_bytes);
        output::Integer(one, "runtime_host_bytes", plan.transaction.host_bytes);
        output::Integer(one, "initializer_retained_host_bytes", plan.initializer.retained_host_bytes);
        output::Integer(one, "initializer_peak_device_bytes", plan.initializer.peak_device_bytes);
        output::Integer(one, "initialization_overlap_device_bytes", plan.peak_device_bytes);
        table.PushBack(output::Value(one, doc.GetAllocator()), doc.GetAllocator());
    }
    doc.AddMember("interfaces", table, doc.GetAllocator());
}
namespace {
void Diagnostics(output::Document& doc, const n::initial_source::Diagnostics& d) {
    output::Integer(doc, "encounters", d.encounters);
    output::Integer(doc, "tasks", d.tasks);
    output::Integer(doc, "pairs", d.pairs);
    output::Integer(doc, "warm_before_tied", d.warm_before_tied);
    output::Integer(doc, "warm_after_tied", d.warm_after_tied);
    output::Integer(doc, "tied_reset", d.tied_reset);
    output::Integer(doc, "warm_positive_main", d.warm_positive_main);
    output::Integer(doc, "warm_zero_main", d.warm_zero_main);
    output::Integer(doc, "warm_negative_main", d.warm_negative_main);
    output::Integer(doc, "changed_gap_corners", d.changed_gap_corners);
    output::Integer(doc, "solid_tagged_nodes", d.solid_tagged_nodes);
    output::Integer(doc, "large_secondaries", d.large_secondaries);
    output::Number(doc, "engine_margin_native", d.engine_margin);
    output::Number(doc, "initial_margin_native", d.initial_margin);
    output::Number(doc, "mean_length_native", d.mean_length);
    output::Number(doc, "edge_average_native", d.edge_average);
    for (unsigned axis = 0; axis < 3; ++axis)
        output::Integer(doc, (std::string("grid_") + char('x' + axis)).c_str(), d.grid[axis]);
}
}
void InitialValues(output::Document& doc, const InitialCensus& row) {
    output::String(doc, "role", row.role == Role::Self ? "self" : "mesh_wall");
    output::Integer(doc, "native_id", row.identity.source.source);
    output::Integer(doc, "source_generation", row.identity.source.topology);
    output::Integer(doc, "runtime_topology", row.identity.source.runtime_topology);
    output::Integer(doc, "nodes", row.identity.nodes);
    output::Integer(doc, "primaries", row.identity.primaries);
    output::Integer(doc, "mains", row.identity.mains);
    output::Integer(doc, "secondaries", row.identity.secondaries);
    output::Boolean(doc, "counts_complete", row.result.counts_complete);
    Diagnostics(doc, row.result.diagnostics);
}
void Failure(output::Document& doc, const std::exception& error) {
    output::Boolean(doc, "completed", false);
    output::String(doc, "error", error.what());
    if (const auto* specific = dynamic_cast<const PreparationError*>(&error)) {
        output::Integer(doc, "initial_status", static_cast<unsigned>(specific->initial.status));
        output::Integer(doc, "initial_row", specific->initial.row);
        output::Integer(doc, "initial_main", specific->initial.main);
        output::Integer(doc, "initial_node", specific->initial.node);
        output::Boolean(doc, "initial_counts_complete", specific->initial.counts_complete);
        Diagnostics(doc, specific->initial.diagnostics);
        output::Integer(doc, "runtime_status", static_cast<unsigned>(specific->transaction.status));
        output::Integer(doc, "runtime_row", specific->transaction.row);
        output::Integer(doc, "runtime_occurrence", specific->transaction.occurrence);
    }
}
} // namespace crash::cases::vehicle_native_contact::test
