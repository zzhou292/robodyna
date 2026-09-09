#include "WallStudyProvenance.h"
#include "CanonicalWallArtifacts.h"
#include "GuidedPlateStudyIO.h"
#include "output/ArtifactIO.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include <cmath>
#include <cstring>
#include <tuple>

namespace crash::case_data {
namespace io=output;
namespace ct=tlfea::contact;
namespace {
using io::Document;using io::Value;
template<class Values> Value Integers(Document& d,const Values& values) {
    Value result(rapidjson::kArrayType);for(auto value:values)result.PushBack(Value().SetUint64(value),d.GetAllocator());return result;
}
void AddChild(Document& d,const char* key,const Document& child) {
    Value value;value.CopyFrom(child,d.GetAllocator());d.AddMember(Value(key,d.GetAllocator()),value,d.GetAllocator());
}
void PushChild(Document& d,Value& array,const Document& child) {
    Value value;value.CopyFrom(child,d.GetAllocator());array.PushBack(value,d.GetAllocator());
}
void Vector(Document& d,const char* key,ct::Vec3 value) {const double a[]{value.x,value.y,value.z};io::FiniteArray(d,key,a,3);}
void Interval(Document& d,const char* key,ct::Q4IntegralInterval value) {const double a[]{value.lower,value.upper};io::FiniteArray(d,key,a,2);}
auto SourceTuple(const WallProvenance& p) {
    return std::tie(p.wall_file,p.combine_file,p.wall_sha256,p.combine_sha256,p.model_archive_reference_sha256,p.generator_sha256,p.obj_sha256);
}
Document Source(const WallProvenance& p) {
    Document d;d.SetObject();io::String(d,"wall_file",p.wall_file);io::String(d,"combine_file",p.combine_file);
    io::String(d,"wall_sha256",p.wall_sha256);io::String(d,"combine_sha256",p.combine_sha256);
    io::String(d,"model_archive_reference_sha256",p.model_archive_reference_sha256);
    io::String(d,"generator_sha256",p.generator_sha256);io::String(d,"obj_sha256",p.obj_sha256);return d;
}
void CheckStudyFootprint(const GuidedStudyData& study,const WallTessellation& wall) {
    io::Require(wall.geometry()&&wall.geometry()->initialized()&&io::Bits(study.config.wall_x)==io::Bits(wall.geometry()->wall_x()),
                "Wall-study exact plane differs from authenticated geometry");
    constexpr unsigned triangles[4][3]={{0,1,2},{0,2,3},{0,1,3},{1,2,3}};
    for(const auto& parent:study.config.contact_reference)for(const auto& nodes:triangles) {
        ct::TriangleGeometry triangle;triangle.face_id=parent.parent.feature_id;
        for(unsigned n=0;n<3;++n) {
            triangle.vertices[n]=parent.reference_projection[nodes[n]];
            triangle.vertex_ids[n]=std::uint64_t(parent.parent.nodes[nodes[n]])+1;
        }
        bool covered=false;
        const auto r=wall.geometry()->ClassifyTriangle(triangle,reference::GuidedPlateData::exposed_clearance,&covered);
        io::Require(r.status==ct::PlanarContactStatus::Ok&&covered,"Study physical reference footprint is not wholly covered by declared wall");
    }
}
GuidedStudyData ValidateInputs(const CanonicalWall& original,const std::string& canonical,const WallTessellation& derived,
                              const std::string& study_bytes) {
    CheckCanonicalWallBinding(original,canonical);
    const auto* m=derived.metadata();const auto* p=derived.source_provenance();
    io::Require(derived.initialized()&&m&&p&&m->source_manifest_sha256==kCanonicalWallManifestSha256&&
                SourceTuple(*p)==SourceTuple(original.provenance()),"Wall-study source is not the authenticated original wall");
    auto study=ParseGuidedPlateStudy(study_bytes);const auto binding=WallTessellationBindingId(m->kind);
    io::Require(binding&&study.config.wall_binding_id==binding&&study.config.qualification_id==kGuidedPlateQualification,
                "Study does not declare this fixed wall variant and guided qualification");
    CheckStudyFootprint(study,derived);return study;
}
Document Expected(const WallTessellation& wall,const GuidedStudyData& study,const std::string& study_bytes) {
    const auto& m=*wall.metadata();const auto mesh=wall.view();Document d;d.SetObject();
    io::String(d,"schema","robo_dyna.wall_study_provenance.v1");
    io::String(d,"scope","Authenticated wall transform and exact completed study bytes; mechanics qualification and canonical replay are separate");
    io::Integer(d,"kind",static_cast<std::uint8_t>(m.kind));io::String(d,"kind_name",WallTessellationName(m.kind));
    io::String(d,"transform_version",m.transform_version);io::String(d,"source_manifest_sha256",m.source_manifest_sha256);
    io::String(d,"study_sha256",io::Sha256(study_bytes));io::Integer(d,"study_bytes",study_bytes.size());
    io::Integer(d,"owner_id",study.config.owner_id);io::Integer(d,"qualification_id",study.config.qualification_id);
    io::Integer(d,"wall_binding_id",study.config.wall_binding_id);io::String(d,"derived_mesh_sha256",m.mesh_sha256);
    io::Integer(d,"original_vertices",m.original_vertices);io::Integer(d,"original_triangles",m.original_triangles);
    io::Integer(d,"derived_vertices",mesh.vertex_count);io::Integer(d,"derived_triangles",mesh.triangle_count);
    io::Integer(d,"original_boundary_edges",m.original_boundary_edges);io::Integer(d,"derived_boundary_edges",m.derived_boundary_edges);
    io::Boolean(d,"exposed_boundary_exact",m.exposed_boundary_exact);
    io::Number(d,"exposed_boundary_displacement_bound_m",m.exposed_boundary_displacement_bound_m);
    io::Number(d,"wall_x_m",wall.geometry()->wall_x());io::Number(d,"footprint_clearance_m",reference::GuidedPlateData::exposed_clearance);
    io::Integer(d,"synthetic_midpoint_id_base",kWallMidpointIdBase);io::Integer(d,"synthetic_face_id_base",kWallDerivedFaceIdBase);
    const auto source=Source(*wall.source_provenance());AddChild(d,"original_source_provenance",source);
    d.AddMember("flipped_source_quads",Integers(d,m.flipped_source_quads),d.GetAllocator());
    Value midpoints(rapidjson::kArrayType),faces(rapidjson::kArrayType);
    for(const auto& point:m.midpoints) {
        io::Require(point.vertex<mesh.vertex_count,"Wall midpoint metadata index is invalid");const auto& vertex=mesh.vertices[point.vertex];
        Document child;child.SetObject();io::Integer(child,"vertex",point.vertex);
        io::Integer(child,"synthetic_source_node_id",vertex.source_node_id);io::Integer(child,"synthetic_assembled_node_id",vertex.assembled_source_node_id);
        child.AddMember("original_edge",Integers(child,point.original_edge),child.GetAllocator());
        child.AddMember("original_source_nodes",Integers(child,point.original_source_nodes),child.GetAllocator());
        child.AddMember("original_assembled_nodes",Integers(child,point.original_assembled_nodes),child.GetAllocator());
        Vector(child,"position_xyz_m",vertex.position);Interval(child,"exact_midpoint_y_m",point.exact_midpoint_y);
        Interval(child,"exact_midpoint_z_m",point.exact_midpoint_z);Vector(child,"coordinate_error_m",point.coordinate_error);
        io::Boolean(child,"exposed_edge",point.exposed_edge);PushChild(d,midpoints,child);
    }
    io::Require(m.faces.size()==mesh.triangle_count,"Wall face lineage does not cover derived mesh");
    for(unsigned i=0;i<mesh.triangle_count;++i) {
        const auto& lineage=m.faces[i];const auto& triangle=mesh.triangles[i];Document child;child.SetObject();
        io::Integer(child,"triangle_index",i);io::Integer(child,"triangle_id",triangle.triangle_id);
        child.AddMember("derived_vertex_indices",Integers(child,triangle.nodes),child.GetAllocator());
        io::Integer(child,"source_quad_id",triangle.source_quad_id);io::Integer(child,"assembled_source_quad_id",triangle.assembled_source_quad_id);
        child.AddMember("original_triangle_ids",Integers(child,lineage.original_triangle_ids),child.GetAllocator());
        io::Integer(child,"original_triangle_count",lineage.original_triangle_count);io::Integer(child,"subtriangle",lineage.subtriangle);
        io::Boolean(child,"connectivity_changed",lineage.connectivity_changed);PushChild(d,faces,child);
    }
    d.AddMember("midpoints",midpoints,d.GetAllocator());d.AddMember("faces",faces,d.GetAllocator());return d;
}
const Value& Required(const Value& object,const char* name) {
    io::Require(object.IsObject(),"Wall-study required field has no object");const Value* result=nullptr;
    for(auto i=object.MemberBegin();i!=object.MemberEnd();++i)if(i->name==name) {
        io::Require(!result,"Duplicate required wall-study provenance field");result=&i->value;
    }
    io::Require(result,"Missing required wall-study provenance field");return *result;
}
void Match(const Value& actual,const Value& expected) {
    if(expected.IsObject()) {
        for(auto i=expected.MemberBegin();i!=expected.MemberEnd();++i)Match(Required(actual,i->name.GetString()),i->value);
    } else if(expected.IsArray()) {
        io::Require(actual.IsArray()&&actual.Size()==expected.Size(),"Wall-study lineage array shape changed");
        for(unsigned i=0;i<expected.Size();++i)Match(actual[i],expected[i]);
    } else if(expected.IsDouble()) {
        io::Require(actual.IsNumber()&&std::isfinite(actual.GetDouble())&&io::Bits(actual.GetDouble())==io::Bits(expected.GetDouble()),
                    "Wall-study floating metadata differs from regenerated geometry");
    } else if(expected.IsUint64()) {
        io::Require(actual.IsUint64()&&actual.GetUint64()==expected.GetUint64(),"Wall-study integer/source identity differs from regenerated geometry");
    } else if(expected.IsBool()) {
        io::Require(actual.IsBool()&&actual.GetBool()==expected.GetBool(),"Wall-study geometry flag changed");
    } else {
        io::Require(expected.IsString()&&actual.IsString()&&actual.GetStringLength()==expected.GetStringLength()&&
            std::memcmp(actual.GetString(),expected.GetString(),expected.GetStringLength())==0,"Wall-study text/hash differs from authenticated metadata");
    }
}
VerifiedWallStudyProvenance Verified(const WallTessellation& wall,const GuidedStudyData& study,const std::string& bytes) {
    return {wall.metadata()->kind,study.config.owner_id,study.config.wall_binding_id,io::Sha256(bytes),
            wall.metadata()->source_manifest_sha256,wall.metadata()->mesh_sha256,wall.view().vertex_count,wall.view().triangle_count};
}
} // namespace
void WriteWallStudyProvenance(const std::filesystem::path& path,const CanonicalWall& original,const std::string& canonical,
                              const WallTessellation& wall,const std::string& study_bytes) {
    const auto study=ValidateInputs(original,canonical,wall,study_bytes);const auto doc=Expected(wall,study,study_bytes);
    rapidjson::StringBuffer buffer;rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    io::Require(doc.Accept(writer),"Wall-study provenance serialization failed");
    io::Require(buffer.GetSize()<kWallStudyProvenanceByteCap,"Wall-study provenance exceeds 1 MiB cap");
    io::WriteBytes(path,std::string(buffer.GetString(),buffer.GetSize())+"\n");
}
VerifiedWallStudyProvenance ParseWallStudyProvenance(const std::string& bytes,const CanonicalWall& original,
    const std::string& canonical,const std::string& study_bytes) {
    io::Require(bytes.size()<=kWallStudyProvenanceByteCap,"Wall-study provenance exceeds 1 MiB cap");Document actual;
    actual.Parse<rapidjson::kParseFullPrecisionFlag|rapidjson::kParseIterativeFlag>(bytes.data(),bytes.size());
    io::Require(!actual.HasParseError()&&actual.IsObject(),"Wall-study provenance parse failed");
    const auto& kind=Required(actual,"kind");io::Require(kind.IsUint64()&&kind.GetUint64()<=2,"Unknown wall-study transform kind");
    WallTessellation regenerated;const auto r=regenerated.Initialize(original,canonical,static_cast<WallTessellationKind>(kind.GetUint64()));
    io::Require(r.status==WallTessellationStatus::Ok,r.diagnostic.c_str());
    const auto study=ValidateInputs(original,canonical,regenerated,study_bytes);const auto expected=Expected(regenerated,study,study_bytes);
    Match(actual,expected);return Verified(regenerated,study,study_bytes);
}
VerifiedWallStudyProvenance ReadWallStudyProvenance(const std::filesystem::path& path,const CanonicalWall& original,
    const std::string& canonical,const std::string& study_bytes) {
    return ParseWallStudyProvenance(io::ReadBounded(path,kWallStudyProvenanceByteCap),original,canonical,study_bytes);
}
} // namespace crash::case_data
