#include "AcceptedReplayData.h"
#include "case/CanonicalWallArtifacts.h"
#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>

namespace crash::output::replay_detail {
namespace {
bool Boolean(const Value& object, const char* name) {
    const auto& value = Member(object, name);
    Require(value.IsBool(), "Guided replay flag has invalid type");
    return value.GetBool();
}
void Array(const Value& value, std::size_t count) {
    Require(value.IsArray() && value.Size() == count, "Guided replay field has invalid extent");
    for (const auto& number : value.GetArray())
        Require(number.IsNumber() && std::isfinite(number.GetDouble()), "Guided replay field is nonfinite or nonnumeric");
}
void Matrix(const Value& object, const char* name, std::size_t rows, std::size_t columns) {
    const auto& value = Member(object, name);
    Require(value.IsArray() && value.Size() == rows, "Guided replay matrix has invalid row count");
    for (const auto& row : value.GetArray()) Array(row, columns);
}
void Certificate(const Value& value) {
    Array(value,4);
    const long double estimate=value[0].GetDouble(),lower=value[1].GetDouble(),upper=value[2].GetDouble(),error=value[3].GetDouble();
    // These are positive force magnitudes/potential. Independently verify the
    // declared radius covers both endpoint distances; no TL mechanics is used.
    // The numerical estimate may lie outside the truth interval; the radius
    // must still cover both distances (the owning Certify contract).
    Require(lower>=0 && upper>=lower && estimate>=0 && error>=0 &&
            error>=std::abs(estimate-lower) && error>=std::abs(upper-estimate),
            "Guided replay certificate columns are inconsistent");
}
void ActiveArea(const Value& object) {
    const auto& value = Member(object,"active_area_m2"); Array(value,2);
    Require(value[0].GetDouble() >= 0 && value[1].GetDouble() >= value[0].GetDouble(), "Invalid guided active-area interval");
}
std::array<std::uint64_t,4> Connectivity(const Value& object, std::size_t nodes) {
    const auto& array = Member(object,"connectivity_zero_based");
    Require(array.IsArray() && array.Size() == 4, "Guided parent requires four node indices");
    std::array<std::uint64_t,4> result{}; std::set<std::uint64_t> unique;
    for (unsigned i=0;i<4;++i) {
        Require(array[i].IsUint64() && array[i].GetUint64() < nodes, "Guided parent node is out of range");
        result[i]=array[i].GetUint64();
        Require(unique.insert(result[i]).second, "Guided parent repeats a node");
    }
    return result;
}
void Phase(const Entry& entry, std::uint64_t base, std::uint64_t attempt, const std::string& phase) {
    Require(attempt && ((phase == "accepted_base" && base == entry.epoch) ||
            (phase == "prepared_candidate_subsequently_committed" && entry.epoch && base == entry.epoch-1)),
            "Guided endpoint field does not identify its accepted configuration");
}
} // namespace

void ReadGuidedConfiguration(Bundle& bundle, const Document& config, const Document& final, const Document& manifest) {
    Require(Boolean(manifest,"contact") && Unsigned(manifest,"owner_id") == bundle.info.owner_id &&
            Unsigned(final,"owner_id") == bundle.info.owner_id, "Guided manifest/final owner or contact scope mismatch");
    for (const char* name : {"shell-intervals.csv","contact-intervals.csv","canonical-wall.manifest.json",
                             "canonical-wall.mesh.json","canonical-wall.obj"})
        Require(bundle.inventory.count(name), "Guided replay lacks a required contributor ledger or canonical wall artifact");
    bundle.qualification_id=Unsigned(config,"qualification_id");
    bundle.wall_binding_id=Unsigned(config,"wall_binding_id");
    Require(bundle.qualification_id && bundle.wall_binding_id &&
            Text(config,"canonical_wall_manifest_sha256") == case_data::kCanonicalWallManifestSha256 &&
            bundle.inventory.at("canonical-wall.manifest.json").hash == case_data::kCanonicalWallManifestSha256,
            "Guided replay is not bound to the original canonical wall and qualification");
    const auto& parents=Member(config,"contact_parent_binding");
    // Guided v1 is the existing two-parent fixture, not a general vehicle format.
    Require(parents.IsArray() && parents.Size()==2, "Guided v1 requires the two immutable physical Q4 parents");
    std::set<std::uint64_t> elements,features;
    for (const auto& parent : parents.GetArray()) {
        ContactParentBinding binding{Unsigned(parent,"parent_element"),Unsigned(parent,"parent_face"),
                                     Unsigned(parent,"feature_id"),Connectivity(parent,bundle.info.node_count)};
        Require(binding.element && binding.feature && elements.insert(binding.element).second &&
                features.insert(binding.feature).second, "Guided parent binding repeats an element or feature");
        bundle.contact_parents.push_back(binding);
    }
    Require(bundle.source_triangles.size()==2*bundle.contact_parents.size(), "Guided display topology does not cover its Q4 parents");
    // Display diagonal is fixed by this schema. It does not define the contact
    // integration or imply that pressure belongs to the display triangles.
    for (std::size_t i=0;i<bundle.contact_parents.size();++i) {
        const auto& p=bundle.contact_parents[i]; const auto& q=p.connectivity;
        const std::array<std::array<std::uint64_t,3>,2> faces{{{{q[0],q[1],q[2]}},{{q[0],q[2],q[3]}}}};
        for (unsigned j=0;j<2;++j) {
            const auto& t=bundle.source_triangles[2*i+j];
            Require(t[5]==p.element && t[7]==p.face && t[8]==j &&
                    t[0]==faces[j][0] && t[1]==faces[j][1] && t[2]==faces[j][2],
                    "Guided contact parent and display source mapping disagree");
        }
    }
}

void CheckGuidedFields(const Bundle& bundle, const Entry& entry, const chrono::ChTriangleMeshConnected& mesh) {
    const auto fields=Json(VerifiedBytes(bundle,entry.mesh.substr(0,entry.mesh.size()-10)+".fields.json"));
    Require(Text(fields,"schema")=="robo_dyna.guided_plate_fields.v1" &&
            Unsigned(fields,"owner_id")==entry.owner && Unsigned(fields,"accepted_epoch")==entry.epoch &&
            Bits(Real(fields,"accepted_time_s"))==Bits(entry.time) &&
            Bits(Real(fields,"fixed_dt_s"))==Bits(bundle.fixed_dt) &&
            Unsigned(fields,"qualification_id")==bundle.qualification_id &&
            Unsigned(fields,"wall_binding_id")==bundle.wall_binding_id, "Guided field identity/time binding mismatch");
    CheckPositionFields(fields,mesh);
    Array(Member(fields,"orientation_wxyz"),4*bundle.info.node_count);
    for (const char* name : {"velocity_xyz_m_per_s","omega_world_xyz_rad_per_s",
                             "reaction_force_xyz_N_at_base","reaction_couple_world_xyz_Nm_at_base"})
        Array(Member(fields,name),3*bundle.info.node_count);
    const bool reactions=Boolean(fields,"reactions_valid");
    const auto reaction_epoch=Unsigned(fields,"reaction_base_epoch");
    const double reaction_time=Real(fields,"reaction_time_s");
    Require(entry.epoch ? (reactions && reaction_epoch==entry.epoch-1 && reaction_time>=0 &&
                            Bits(reaction_time+bundle.fixed_dt)==Bits(entry.time))
                        : (!reactions && reaction_epoch==0 && reaction_time==0),
            "Guided base reactions are not associated with the preceding accepted interval");
    const auto base=Unsigned(fields,"element_evaluation_base_epoch"), attempt=Unsigned(fields,"element_evaluation_attempt");
    const auto phase=Text(fields,"element_evaluation_phase"); Phase(entry,base,attempt,phase);
    Require(Unsigned(fields,"contact_evaluation_base_epoch")==base &&
            Unsigned(fields,"contact_evaluation_attempt")==attempt && Text(fields,"contact_evaluation_phase")==phase,
            "Guided contributor endpoint phases disagree");
    const auto& elements=Member(fields,"elements"); const auto& parents=Member(fields,"contact_parents");
    Require(elements.IsArray() && elements.Size()==bundle.contact_parents.size() &&
            parents.IsArray() && parents.Size()==bundle.contact_parents.size(), "Guided contributor parent count mismatch");
    for (unsigned i=0;i<elements.Size();++i) {
        const auto& element=elements[i]; const auto& parent=parents[i]; const auto& binding=bundle.contact_parents[i];
        Require(Unsigned(element,"parent_element")==binding.element && Unsigned(parent,"parent_element")==binding.element &&
                Unsigned(parent,"parent_face")==binding.face && Unsigned(parent,"feature_id")==binding.feature &&
                Connectivity(parent,bundle.info.node_count)==binding.connectivity &&
                Unsigned(parent,"base_epoch")==base && Unsigned(parent,"attempt")==attempt,
                "Guided contributor source or endpoint association mismatch");
        (void)Real(element,"elastic_energy_J"); (void)Real(element,"bending_energy_J");
        Matrix(element,"force_world_N",4,3); Matrix(element,"couple_world_Nm",4,3);
        Matrix(element,"gauss_strain",4,12); Matrix(element,"gauss_resultant",4,12);
        const bool covered=Boolean(parent,"covered"),valid=Boolean(parent,"valid");
        Require(covered && valid, "Guided v1 requires both fully covered valid parents");
        Matrix(parent,"force_world_N",4,3); Matrix(parent,"couple_world_Nm",4,3);
        Matrix(parent,"nodal_force_magnitude_N",4,4);
        for (const auto& value : Member(parent,"nodal_force_magnitude_N").GetArray()) Certificate(value);
        Certificate(Member(parent,"resultant_magnitude_N")); Certificate(Member(parent,"potential_J")); ActiveArea(parent);
        for (const char* name : {"leaf_count","visited","deepest_leaf"}) (void)Unsigned(parent,name);
    }
    const auto& contact=Member(fields,"contact_result");
    for (const char* name : {"wall_reaction_N","wall_moment_Nm","force_error_N","wall_moment_error_Nm"})
        Array(Member(contact,name),3);
    for (const char* name : {"force_error_N","wall_moment_error_Nm"})
        for (const auto& value : Member(contact,name).GetArray())
            Require(value.GetDouble()>=0, "Guided contact error bound is negative");
    Certificate(Member(contact,"potential_J")); ActiveArea(contact);
    Require(Real(contact,"maximum_penetration_m")>=0, "Guided contact penetration is negative");
    // This verifies record structure/identity, not an independent re-evaluation
    // of strain, section stress, contact certificates or any interval mechanics.
}

void CheckGuidedWall(const Bundle& bundle, const chrono::ChTriangleMeshConnected& mesh) {
    const auto bytes=VerifiedBytes(bundle,"canonical-wall.manifest.json");
    std::istringstream input(bytes); case_data::CanonicalWall wall;
    const auto report=wall.Load(input);
    Require(report.status==case_data::WallStatus::Ok, report.message);
    case_data::CheckCanonicalWallBinding(wall,bytes);
    Require(wall.vertices().size()==62 && wall.triangles().size()==100 && wall.source_quads().size()==46 &&
            mesh.GetCoordsVertices().size()==wall.vertices().size() &&
            mesh.GetIndicesVertices().size()==wall.triangles().size(), "Guided wall topology is not the original canonical mesh");
    for (std::size_t i=0;i<wall.vertices().size();++i)
        for (unsigned j=0;j<3;++j)
            Require(Bits(mesh.GetCoordsVertices()[i][j])==Bits(wall.vertices()[i].position_m[j]), "Guided wall coordinates changed");
    for (std::size_t i=0;i<wall.triangles().size();++i)
        for (unsigned j=0;j<3;++j)
            Require(mesh.GetIndicesVertices()[i][j]==static_cast<int>(wall.triangles()[i].vertex_indices[j]),
                    "Guided wall connectivity changed");
}
} // namespace crash::output::replay_detail
