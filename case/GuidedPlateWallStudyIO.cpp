#include "GuidedPlateWallStudyIO.h"
#include "GuidedPlateStudyIO.h"
#include "GuidedPlateComparisonFields.h"
#include "GuidedPlateContactProtocol.h"
#include "WallStudyProvenance.h"
#include "CanonicalWallArtifacts.h"
#include "output/ArtifactIO.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include <sstream>

namespace crash::case_data {
namespace io=output;
namespace {
void Identity(io::Document& d,const char* name,const VerifiedWallStudyProvenance& verified,const std::string& sidecar_sha) {
    io::Document item;item.SetObject();
    io::Integer(item,"wall_kind",static_cast<unsigned>(verified.kind));io::String(item,"wall_kind_name",WallTessellationName(verified.kind));
    io::Integer(item,"owner_id",verified.owner_id);io::Integer(item,"wall_binding_id",verified.wall_binding_id);
    io::String(item,"study_sha256",verified.study_sha256);io::String(item,"provenance_sha256",sidecar_sha);
    io::String(item,"mesh_sha256",verified.derived_mesh_sha256);io::Integer(item,"vertices",verified.derived_vertices);
    io::Integer(item,"triangles",verified.derived_triangles);
    io::Value copy;copy.CopyFrom(item,d.GetAllocator());d.AddMember(io::Value(name,d.GetAllocator()),copy,d.GetAllocator());
}
} // namespace
GuidedStudyComparison CompareAndWriteGuidedPlateWallStudies(const GuidedPlateWallStudyPaths& paths) {
    namespace fs=std::filesystem;
    io::Require(!paths.report.empty()&&!fs::exists(fs::symlink_status(paths.report)),"Wall comparison output must be new");
    const auto parent=paths.report.has_parent_path()?paths.report.parent_path():fs::path(".");
    io::Require(fs::is_directory(parent),"Wall comparison output parent must exist");
    const auto wall_bytes=ReadPinnedWallManifest(paths.wall.string());std::istringstream stream(wall_bytes);CanonicalWall wall;
    const auto loaded=wall.Load(stream);io::Require(loaded.status==WallStatus::Ok,loaded.message);
    // Read each input exactly once. Hash and validate these SAME bytes, never
    // reopen paths after authentication to select data for comparison.
    const auto derived_bytes=io::ReadBounded(paths.derived_study,kGuidedStudyByteCap);
    const auto canonical_bytes=io::ReadBounded(paths.canonical_study,kGuidedStudyByteCap);
    const auto derived_sidecar=io::ReadBounded(paths.derived_provenance,kWallStudyProvenanceByteCap);
    const auto canonical_sidecar=io::ReadBounded(paths.canonical_provenance,kWallStudyProvenanceByteCap);
    const auto derived_proof=ParseWallStudyProvenance(derived_sidecar,wall,wall_bytes,derived_bytes);
    const auto canonical_proof=ParseWallStudyProvenance(canonical_sidecar,wall,wall_bytes,canonical_bytes);
    io::Require((derived_proof.kind==WallTessellationKind::FlipConvexPairs||derived_proof.kind==WallTessellationKind::UniformFour)&&
        canonical_proof.kind==WallTessellationKind::Original&&derived_proof.source_manifest_sha256==canonical_proof.source_manifest_sha256,
        "Wall comparison requires an authenticated derived response followed by the authenticated Original response");
    const auto derived=ParseGuidedPlateStudy(derived_bytes),canonical=ParseGuidedPlateStudy(canonical_bytes);
    GuidedStudyComparison comparison;std::string error;
    if(!CompareGuidedPlateWallStudies(derived,canonical,comparison,error))throw std::runtime_error(error);
    io::Document d;d.SetObject();io::String(d,"schema","robo_dyna.guided_plate_wall_comparison.v1");
    io::String(d,"scope","Same-fixed-step wall response comparison; exact Study/sidecar bytes and original wall transforms authenticated. Mechanics execution/history and replay qualification remain separate.");
    io::String(d,"source_manifest_sha256",canonical_proof.source_manifest_sha256);
    Identity(d,"derived",derived_proof,io::Sha256(derived_sidecar));Identity(d,"canonical",canonical_proof,io::Sha256(canonical_sidecar));
    io::Integer(d,"qualification_id",canonical.config.qualification_id);io::String(d,"experiment_sha256",canonical.config.experiment_sha256);
    io::Integer(d,"base_steps",canonical.config.base_steps);io::Integer(d,"refinement",canonical.config.refinement);
    const auto* backend=GuidedContactBackendName(canonical.config.integration_backend);
    io::Require(backend&&io::contact_metadata::Known(backend),"Invalid comparison backend");
    io::String(d,io::contact_metadata::BackendField,backend);
    io::Number(d,"fixed_dt_s",canonical.config.fixed_dt);io::Number(d,"horizon_s",canonical.config.horizon);
    io::String(d,"relative_scale","canonical response (second argument)");comparison_io::Append(d,comparison);
    rapidjson::StringBuffer buffer;rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    io::Require(d.Accept(writer)&&buffer.GetSize()<kGuidedStudyByteCap,"Wall comparison serialization exceeds its bounded report");
    io::WriteBytes(paths.report,std::string(buffer.GetString(),buffer.GetSize())+"\n");return comparison;
}
} // namespace crash::case_data
