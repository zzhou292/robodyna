#include "CanonicalWall.h"
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <new>
#include <set>
#include <sstream>
#include <utility>

namespace crash::case_data {
struct CanonicalWall::Data {
    std::vector<WallVertex> vertices;
    std::vector<WallTriangle> triangles;
    std::vector<WallSourceQuad> quads;
    std::vector<WallStitch> stitches;
    WallReactionGroups groups;
    WallProvenance provenance;
    std::array<double,3> normal{};
    std::array<std::array<double,3>,2> bounds{};
    double area = 0;
};
namespace {
using V = rapidjson::Value;
using Point = std::array<double,3>;
using Edge = std::pair<std::uint32_t,std::uint32_t>;
struct Invalid { WallStatus status; const char* message; };
void Require(bool condition, const char* message, WallStatus status = WallStatus::InvalidData) {
    if (!condition) throw Invalid{status,message};
}
const V& Member(const V& object, const char* key) {
    Require(object.IsObject() && object.HasMember(key), "Missing canonical wall member", WallStatus::InvalidSchema);
    return object[key];
}
const V& Array(const V& value, std::size_t size) {
    Require(value.IsArray() && value.Size()==size, "Invalid canonical array size/type"); return value;
}
std::uint64_t Id(const V& value, bool allow_zero = false) {
    Require(value.IsUint64(), "Source IDs/counts require exact unsigned integer JSON tokens");
    auto id=value.GetUint64(); Require(allow_zero || id!=0, "Zero source ID"); return id;
}
double Number(const V& value) {
    Require(value.IsNumber(), "Coordinate or scalar is not a JSON number");
    const double number=value.GetDouble(); Require(std::isfinite(number), "Nonfinite wall number"); return number;
}
Point Coordinates(const V& value) {
    Array(value,3); return {Number(value[0]),Number(value[1]),Number(value[2])};
}
std::string String(const V& value) {
    Require(value.IsString() && value.GetStringLength()!=0, "Missing string metadata");
    return {value.GetString(),value.GetStringLength()};
}
void Equals(const V& value, const char* expected) {
    Require(value.IsString() && std::string(value.GetString(),value.GetStringLength())==expected,
            "Unexpected canonical wall schema/role", WallStatus::InvalidSchema);
}
std::string Hash(const V& value) {
    const auto text=String(value);
    Require(text.size()==64 && std::all_of(text.begin(),text.end(),[](char c){return (c>='0'&&c<='9')||(c>='a'&&c<='f');}),
            "Invalid declared SHA256"); return text;
}
std::uint64_t Offset(std::uint64_t id, std::uint64_t offset) {
    Require(id<=UINT64_MAX-offset, "Assembled source ID overflow"); return id+offset;
}
template<std::size_t N> std::array<std::uint64_t,N> IdArray(const V& value) {
    Array(value,N); std::array<std::uint64_t,N> ids{};
    for (std::size_t i=0;i<N;++i) ids[i]=Id(value[static_cast<rapidjson::SizeType>(i)]);
    Require(std::set<std::uint64_t>(ids.begin(),ids.end()).size()==N,"Repeated node in source entity"); return ids;
}
std::vector<std::uint64_t> IdList(const V& value, std::size_t cap) {
    Require(value.IsArray() && value.Size()<=cap,"Invalid or excessive ID list");
    std::vector<std::uint64_t> result; result.reserve(value.Size());
    std::set<std::uint64_t> seen;
    for (const auto& item:value.GetArray()) { const auto id=Id(item); Require(seen.insert(id).second,"Duplicate source ID in list"); result.push_back(id); }
    return result;
}
void CheckTree(const V& root) {
    // The bundled iterative parser avoids recursive parsing of hostile nesting;
    // this bounded traversal rejects deep/duplicate-key documents before schema reads.
    std::vector<std::pair<const V*,unsigned>> pending{{&root,0}};
    std::size_t visited=0;
    while (!pending.empty()) {
        const auto entry=pending.back(); pending.pop_back(); const auto& value=*entry.first;
        Require(entry.second<=16 && ++visited<=20000,"JSON structure exceeds canonical limits",WallStatus::ResourceLimit);
        if (value.IsObject()) {
            std::set<std::string> names;
            for (auto it=value.MemberBegin();it!=value.MemberEnd();++it) {
                Require(names.emplace(it->name.GetString(),it->name.GetStringLength()).second,"Duplicate JSON member",WallStatus::InvalidSchema);
                pending.push_back({&it->value,entry.second+1});
            }
        } else if(value.IsArray()) for(const auto& item:value.GetArray()) pending.push_back({&item,entry.second+1});
    }
}
double Cross(const Point& a,const Point& b,const Point& c) {
    const double value=(b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1]);
    Require(std::isfinite(value),"Wall geometry arithmetic overflow"); return value;
}
double Parameter(const Point& a,const Point& b,const Point& p,double tolerance) {
    const double y=b[1]-a[1],z=b[2]-a[2],length=std::hypot(y,z);
    Require(std::isfinite(length) && length>0,"Zero or invalid wall edge");
    const double t=((p[1]-a[1])*(y/length)+(p[2]-a[2])*(z/length))/length;
    Require(std::isfinite(t),"Invalid wall edge parameter");
    return std::fabs(Cross(a,b,p))/length<=tolerance ? t : -1;
}
bool CloseArea(double a,double b) {
    return std::isfinite(a) && std::isfinite(b) && std::fabs(a-b)<=1e-14+1e-12*std::max(std::fabs(a),std::fabs(b));
}
template<class T> const T& Empty() { static const T value{}; return value; }
}  // namespace

CanonicalWall::CanonicalWall()=default;
CanonicalWall::~CanonicalWall()=default;
bool CanonicalWall::loaded()const noexcept{return bool(data_);}
const std::vector<WallVertex>& CanonicalWall::vertices()const noexcept{return data_?data_->vertices:Empty<std::vector<WallVertex>>();}
const std::vector<WallTriangle>& CanonicalWall::triangles()const noexcept{return data_?data_->triangles:Empty<std::vector<WallTriangle>>();}
const std::vector<WallSourceQuad>& CanonicalWall::source_quads()const noexcept{return data_?data_->quads:Empty<std::vector<WallSourceQuad>>();}
const std::vector<WallStitch>& CanonicalWall::stitching()const noexcept{return data_?data_->stitches:Empty<std::vector<WallStitch>>();}
const WallReactionGroups& CanonicalWall::reaction_groups()const noexcept{return data_?data_->groups:Empty<WallReactionGroups>();}
const WallProvenance& CanonicalWall::provenance()const noexcept{return data_?data_->provenance:Empty<WallProvenance>();}
std::array<double,3> CanonicalWall::front_normal()const noexcept{return data_?data_->normal:Point{};}
std::array<std::array<double,3>,2> CanonicalWall::bounds_m()const noexcept{return data_?data_->bounds:std::array<Point,2>{};}
double CanonicalWall::area_m2()const noexcept{return data_?data_->area:0;}

WallReport CanonicalWall::LoadFile(const std::string& path,WallLimits limits) {
    if(data_)return {WallStatus::AlreadyLoaded,"Wall is immutable after publication"};
    std::ifstream input(path,std::ios::binary);
    if(!input)return {WallStatus::IoError,"Required canonical wall manifest could not be opened"};
    return Load(input,limits);
}
WallReport CanonicalWall::Load(std::istream& input,WallLimits limits) {
    if(data_)return {WallStatus::AlreadyLoaded,"Wall is immutable after publication"};
    try {
        Require(limits.max_vertices>0 && limits.max_vertices<=64 && limits.max_triangles>0 && limits.max_triangles<=100 &&
                limits.max_source_quads>0 && limits.max_source_quads<=46 && limits.max_json_bytes>0 && limits.max_json_bytes<=1024*1024,
                "Invalid canonical wall resource caps",WallStatus::ResourceLimit);
        // Bound source bytes BEFORE any JSON DOM allocation. No unbounded read or
        // downloaded include resolution. IStreamWrapper parses this bounded input.
        std::string bytes; char chunk[4096];
        while(input) {
            input.read(chunk,sizeof(chunk)); const auto count=input.gcount();
            Require(count>=0 && static_cast<std::size_t>(count)<=limits.max_json_bytes-bytes.size(),
                    "Canonical wall JSON exceeds byte cap",WallStatus::ResourceLimit);
            bytes.append(chunk,static_cast<std::size_t>(count));
        }
        Require(!input.bad() && input.eof(),"Canonical wall stream read failed",WallStatus::IoError);
        std::istringstream bounded(bytes); rapidjson::IStreamWrapper stream(bounded); rapidjson::Document document;
        document.ParseStream<rapidjson::kParseFullPrecisionFlag|rapidjson::kParseIterativeFlag|rapidjson::kParseValidateEncodingFlag>(stream);
        if(document.HasParseError())return {WallStatus::ParseError,"Invalid canonical wall JSON",document.GetErrorOffset()};
        CheckTree(document);
        Equals(Member(document,"schema"),"tlfea.yaris_fixed_wall.v1");
        Equals(Member(document,"scope"),"Fixed wall geometry only; no vehicle or material dynamics imported");
        Equals(Member(document,"output_length_unit"),"m"); Equals(Member(document,"source_length_unit"),"mm");
        Require(Number(Member(document,"length_scale"))==.001,"Unexpected wall unit conversion");
        auto result=std::make_unique<Data>();
        const auto& contact=Member(document,"contact");
        Equals(Member(contact,"representation"),"fixed_triangle_mesh");
        Require(Id(Member(contact,"dynamic_wall_dofs"),true)==0,"Dynamic wall DOFs are not admitted");
        const auto& analytic=Member(contact,"analytic_force_generation");
        Require(analytic.IsBool() && !analytic.GetBool(),"Analytic force generation must be disabled");
        result->normal=Coordinates(Member(contact,"front_normal"));
        Require(result->normal==Point{-1,0,0} && Coordinates(Member(contact,"velocity_m_per_s"))==Point{},"Wall must be fixed and face -X");
        Require(Number(Member(contact,"additional_wall_offset_m"))==0,"Wall offset is not admitted");
        Require(Number(Member(contact,"friction"))>=0 && Number(Member(contact,"source_display_thickness_m"))>=0,"Invalid source contact metadata");
        const auto& source=Member(document,"source"); auto& provenance=result->provenance;
        Equals(Member(source,"wall_file"),"source/wall.key"); Equals(Member(source,"combine_file"),"source/combine.key");
        provenance.wall_file=String(Member(source,"wall_file")); provenance.combine_file=String(Member(source,"combine_file"));
        provenance.wall_sha256=Hash(Member(source,"wall_sha256")); provenance.combine_sha256=Hash(Member(source,"combine_sha256"));
        provenance.model_archive_reference_sha256=Hash(Member(source,"model_archive_reference_sha256"));
        provenance.generator_sha256=Hash(Member(Member(document,"generator"),"sha256"));
        provenance.obj_sha256=Hash(Member(Member(Member(document,"artifacts"),"wall.obj"),"sha256"));
        const auto& transform=Member(document,"transform");
        const auto node_offset=Id(Member(transform,"node_id_offset"),true),quad_offset=Id(Member(transform,"element_id_offset"),true),part_offset=Id(Member(transform,"part_id_offset"),true);
        Coordinates(Member(transform,"translation_mm")); Id(Member(transform,"transformation_id")); Id(Member(transform,"rigid_wall_id_offset"),true);
        const auto& vertices=Member(document,"vertices"); const auto& triangles=Member(document,"triangles"); const auto& quads=Member(document,"source_quads");
        Require(vertices.IsArray() && vertices.Size()>0 && vertices.Size()<=limits.max_vertices && triangles.IsArray() && triangles.Size()>0 &&
                triangles.Size()<=limits.max_triangles && quads.IsArray() && quads.Size()>0 && quads.Size()<=limits.max_source_quads,
                "Invalid wall entity counts/capacity",WallStatus::ResourceLimit);
        const auto& counts=Member(document,"counts");
        Require(Id(Member(counts,"collision_vertices"))==vertices.Size() && Id(Member(counts,"source_nodes"))==vertices.Size() &&
                Id(Member(counts,"collision_triangles"))==triangles.Size() && Id(Member(counts,"source_quads"))==quads.Size(),"Manifest count mismatch");
        std::map<std::uint64_t,std::size_t> node_map,quad_map;
        std::set<std::uint64_t> assembled_nodes,assembled_quads; std::set<Point> positions;
        for(const auto& item:vertices.GetArray()) {
            WallVertex vertex; const auto index=Id(Member(item,"vertex_index"),true);
            Require(index==result->vertices.size(),"Vertex index does not match canonical array order"); vertex.vertex_index=static_cast<std::uint32_t>(index);
            vertex.position_m=Coordinates(Member(item,"position_m")); Require(vertex.position_m[0]==.05,"Canonical wall must be exactly planar at X=.05 m");
            vertex.source_node_id=Id(Member(item,"source_node_id")); vertex.assembled_source_node_id=Id(Member(item,"assembled_source_node_id"));
            Require(vertex.assembled_source_node_id==Offset(vertex.source_node_id,node_offset),"Assembled node ID mapping mismatch");
            Require(node_map.emplace(vertex.source_node_id,index).second && assembled_nodes.insert(vertex.assembled_source_node_id).second && positions.insert(vertex.position_m).second,
                    "Duplicate node ID or coincident wall vertex"); result->vertices.push_back(vertex);
        }
        auto point=[&](std::uint64_t id)->const Point& {
            const auto found=node_map.find(id); Require(found!=node_map.end(),"Source node reference is missing"); return result->vertices[found->second].position_m;
        };
        std::vector<std::set<std::uint64_t>> allowed;
        std::vector<double> quad_area,triangle_area;
        for(const auto& item:quads.GetArray()) {
            WallSourceQuad quad; quad.source_quad_id=Id(Member(item,"source_quad_id")); quad.assembled_source_quad_id=Id(Member(item,"assembled_source_quad_id"));
            quad.source_part_id=Id(Member(item,"source_part_id")); quad.assembled_source_part_id=Id(Member(item,"assembled_source_part_id"));
            quad.source_node_ids=IdArray<4>(Member(item,"source_node_ids"));
            Require(quad.assembled_source_quad_id==Offset(quad.source_quad_id,quad_offset) && quad.assembled_source_part_id==Offset(quad.source_part_id,part_offset),"Assembled quad/part ID mismatch");
            Require(quad_map.emplace(quad.source_quad_id,result->quads.size()).second && assembled_quads.insert(quad.assembled_source_quad_id).second,"Duplicate source quad");
            for(unsigned i=0;i<4;++i)Require(Cross(point(quad.source_node_ids[i]),point(quad.source_node_ids[(i+1)%4]),point(quad.source_node_ids[(i+2)%4]))>0,"Source quad must be convex with original +X winding");
            quad_area.push_back(.5*(Cross(point(quad.source_node_ids[0]),point(quad.source_node_ids[1]),point(quad.source_node_ids[2]))+
                                      Cross(point(quad.source_node_ids[0]),point(quad.source_node_ids[2]),point(quad.source_node_ids[3]))));
            Require(std::isfinite(quad_area.back()) && quad_area.back()>0,"Invalid source quad area sum");
            allowed.emplace_back(quad.source_node_ids.begin(),quad.source_node_ids.end()); result->quads.push_back(quad);
        }
        triangle_area.resize(result->quads.size());
        std::vector<unsigned> triangle_count(result->quads.size());
        const auto& stitching=Member(document,"stitching"); const double tolerance=Number(Member(stitching,"tolerance_m"));
        Require(tolerance>0 && tolerance<=1e-10,"Invalid canonical stitching tolerance");
        const auto& stitches=Member(stitching,"edges"); Require(stitches.IsArray() && stitches.Size()<=4*quads.Size(),"Invalid stitching edges");
        std::set<std::array<std::uint64_t,3>> stitched_edges;
        for(const auto& item:stitches.GetArray()) {
            WallStitch stitch; stitch.source_quad_id=Id(Member(item,"source_quad_id")); stitch.source_edge=IdArray<2>(Member(item,"source_edge"));
            stitch.inserted_source_node_ids=IdList(Member(item,"inserted_source_node_ids"),vertices.Size());
            Require(!stitch.inserted_source_node_ids.empty(),"Empty declared stitch"); const auto q=quad_map.find(stitch.source_quad_id);
            Require(q!=quad_map.end(),"Stitch references missing source quad"); const auto& ids=result->quads[q->second].source_node_ids;
            bool edge=false;for(unsigned i=0;i<4;++i)edge|=ids[i]==stitch.source_edge[0]&&ids[(i+1)%4]==stitch.source_edge[1];
            Require(edge && stitched_edges.insert({stitch.source_quad_id,stitch.source_edge[0],stitch.source_edge[1]}).second,"Invalid/duplicate source stitch edge");
            double previous=0;
            for(auto id:stitch.inserted_source_node_ids) {
                const double t=Parameter(point(stitch.source_edge[0]),point(stitch.source_edge[1]),point(id),tolerance);
                Require(t>previous && t<1 && allowed[q->second].insert(id).second,"Invalid or unordered inserted stitch node"); previous=t;
            }
            result->stitches.push_back(std::move(stitch));
        }
        struct Use { std::size_t triangle; std::uint32_t a,b; };
        std::map<Edge,std::vector<Use>> edges; std::set<std::array<std::uint32_t,3>> faces; std::set<std::uint64_t> triangle_ids;
        std::vector<bool> used(vertices.Size());
        for(const auto& item:triangles.GetArray()) {
            WallTriangle triangle; triangle.triangle_id=Id(Member(item,"triangle_id")); triangle.source_quad_id=Id(Member(item,"source_quad_id"));
            triangle.assembled_source_quad_id=Id(Member(item,"assembled_source_quad_id")); triangle.source_node_ids=IdArray<3>(Member(item,"source_node_ids"));
            Require(triangle_ids.insert(triangle.triangle_id).second,"Duplicate collision triangle ID"); const auto q=quad_map.find(triangle.source_quad_id);
            Require(q!=quad_map.end() && triangle.assembled_source_quad_id==result->quads[q->second].assembled_source_quad_id,"Triangle parent quad mapping mismatch");
            const auto& indices=Array(Member(item,"vertex_indices"),3);
            for(unsigned i=0;i<3;++i) {
                const auto index=Id(indices[i],true); Require(index<vertices.Size(),"Triangle vertex index out of range");
                triangle.vertex_indices[i]=static_cast<std::uint32_t>(index); used[index]=true;
                Require(result->vertices[index].source_node_id==triangle.source_node_ids[i] && allowed[q->second].count(triangle.source_node_ids[i]),"Triangle source node mapping mismatch");
            }
            auto sorted=triangle.vertex_indices; std::sort(sorted.begin(),sorted.end()); Require(faces.insert(sorted).second,"Duplicate collision triangle geometry");
            const double area=-.5*Cross(point(triangle.source_node_ids[0]),point(triangle.source_node_ids[1]),point(triangle.source_node_ids[2]));
            Require(area>0 && std::isfinite(area),"Triangle must be nondegenerate and face -X"); triangle_area[q->second]+=area; result->area+=area;
            Require(std::isfinite(triangle_area[q->second]) && std::isfinite(result->area),"Collision triangle area sum overflow"); ++triangle_count[q->second];
            for(unsigned i=0;i<3;++i) {
                const auto a=triangle.vertex_indices[i],b=triangle.vertex_indices[(i+1)%3]; edges[std::minmax(a,b)].push_back({result->triangles.size(),a,b});
            }
            result->triangles.push_back(triangle);
        }
        for(std::size_t i=0;i<quad_area.size();++i)Require(triangle_count[i]>=2 && CloseArea(quad_area[i],triangle_area[i]),"Collision triangles do not preserve source quad area");
        Require(std::all_of(used.begin(),used.end(),[](bool v){return v;}),"Unused canonical vertex");
        std::vector<std::vector<std::size_t>> neighbors(triangles.Size()); std::vector<unsigned> boundary_degree(vertices.Size());
        std::set<std::array<std::uint64_t,2>> boundary; std::size_t interior=0;
        for(const auto& entry:edges) {
            const auto& uses=entry.second; Require(uses.size()==1 || uses.size()==2,"Nonmanifold wall edge");
            if(uses.size()==2) {
                Require(uses[0].a==uses[1].b && uses[0].b==uses[1].a,"Shared wall edge winding mismatch"); ++interior;
                neighbors[uses[0].triangle].push_back(uses[1].triangle); neighbors[uses[1].triangle].push_back(uses[0].triangle);
            } else {
                ++boundary_degree[entry.first.first]; ++boundary_degree[entry.first.second];
                auto a=result->vertices[entry.first.first].source_node_id,b=result->vertices[entry.first.second].source_node_id;
                boundary.insert({std::min(a,b),std::max(a,b)});
            }
            const auto& a=result->vertices[entry.first.first].position_m; const auto& b=result->vertices[entry.first.second].position_m;
            for(std::size_t i=0;i<vertices.Size();++i)if(i!=entry.first.first&&i!=entry.first.second) {
                const double t=Parameter(a,b,result->vertices[i].position_m,tolerance); Require(!(t>0&&t<1),"Hanging node on collision edge");
            }
        }
        for(auto degree:boundary_degree)Require(degree==0||degree==2,"Wall boundary is not a closed simple loop");
        std::set<std::size_t> reached{0}; std::vector<std::size_t> queue{0};
        while(!queue.empty()) { const auto i=queue.back();queue.pop_back();for(auto next:neighbors[i])if(reached.insert(next).second)queue.push_back(next); }
        Require(reached.size()==triangles.Size() && vertices.Size()+triangles.Size()==edges.size()+1,"Wall must form one connected disk");
        for(auto a=edges.begin();a!=edges.end();++a)for(auto b=std::next(a);b!=edges.end();++b) {
            if(a->first.first==b->first.first||a->first.first==b->first.second||a->first.second==b->first.first||a->first.second==b->first.second)continue;
            const auto& p=result->vertices[a->first.first].position_m;const auto& q=result->vertices[a->first.second].position_m;
            const auto& r=result->vertices[b->first.first].position_m;const auto& s=result->vertices[b->first.second].position_m;
            const double c1=Cross(p,q,r),c2=Cross(p,q,s),c3=Cross(r,s,p),c4=Cross(r,s,q);
            Require(!(((c1<0&&c2>0)||(c1>0&&c2<0))&&((c3<0&&c4>0)||(c3>0&&c4<0))),"Crossing nonadjacent wall edges");
        }
        const auto& validation=Member(document,"validation"); Require(CloseArea(result->area,Number(Member(validation,"area_m2"))),"Declared wall area mismatch");
        Require(Id(Member(validation,"euler_characteristic"))==1 && Id(Member(validation,"unique_edges"))==edges.size() &&
                Id(Member(validation,"interior_edges"),true)==interior,"Declared topology diagnostics mismatch");
        const auto& declared_boundary=Member(validation,"boundary_edges"); Require(declared_boundary.IsArray()&&declared_boundary.Size()==boundary.size(),"Declared boundary count mismatch");
        std::set<std::array<std::uint64_t,2>> expected_boundary;
        for(const auto& item:declared_boundary.GetArray()) { auto ids=IdArray<2>(item);std::sort(ids.begin(),ids.end());Require(expected_boundary.insert(ids).second,"Duplicate boundary source edge"); }
        Require(expected_boundary==boundary,"Boundary source mapping mismatch");
        const auto& bounds=Array(Member(document,"bounds_m"),2); result->bounds={Coordinates(bounds[0]),Coordinates(bounds[1])};
        std::array<Point,2> actual{result->vertices[0].position_m,result->vertices[0].position_m};
        for(const auto& v:result->vertices)for(unsigned i=0;i<3;++i){actual[0][i]=std::min(actual[0][i],v.position_m[i]);actual[1][i]=std::max(actual[1][i],v.position_m[i]);}
        Require(result->bounds==actual,"Declared binary64 bounds mismatch");
        const auto& groups=Member(document,"reaction_groups");
        result->groups.whole_wall_triangle_ids=IdList(Member(groups,"whole_wall_triangle_ids"),triangles.Size());
        result->groups.source_segment_set_1001_triangle_ids=IdList(Member(groups,"source_segment_set_1001_triangle_ids"),triangles.Size());
        Require(std::set<std::uint64_t>(result->groups.whole_wall_triangle_ids.begin(),result->groups.whole_wall_triangle_ids.end())==triangle_ids,"Whole-wall reaction group is incomplete");
        for(auto id:result->groups.source_segment_set_1001_triangle_ids)Require(triangle_ids.count(id),"Reaction group references missing triangle");
        data_=std::move(result); return {WallStatus::Ok,"OK"};
    } catch(const Invalid& error) { return {error.status,error.message}; }
      catch(const std::bad_alloc&) { return {WallStatus::ResourceLimit,"Canonical wall allocation failed"}; }
      catch(const std::ios_base::failure&) { return {WallStatus::IoError,"Canonical wall stream read failed"}; }
}
}  // namespace crash::case_data
