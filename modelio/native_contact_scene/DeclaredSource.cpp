#include "DeclaredSource.h"
#include "output/BoundedArrayJson.h"
#include <map>
#include <set>
namespace crash::modelio::native_scene {
namespace {
using namespace output;
using namespace output::array_json;
const Value& Field(const Value& v,const char* key) {
    Require(v.IsObject()&&v.HasMember(key),"Missing declared scene field");return v[key];
}
double RealField(const Value& v,const char* key){return Real(Field(v,key));}
std::uint64_t Id(const Value& v){const auto id=UInt(v);Require(id>0&&id<=UINT32_MAX,"Scene source ID out of range");return id;}
tl::math::Vec3 Vector(const Value& v) {
    Require(v.IsArray()&&v.Size()==3,"Scene vector must contain three scalars");return {Real(v[0]),Real(v[1]),Real(v[2])};
}
void SameNumber(const Value& a,const Value& b){Require(Bits(Real(a))==Bits(Real(b)),"Exported material/run value differs from source JSON");}
void Materials(DeclaredData& out,const Value& scene,const Value& original) {
    const auto& m=Field(scene,"material");const auto& raw=Field(original,"material");
    for(const auto* value:{&m,&raw})Keys(*value,{"density_tonne_mm3","young_n_mm2","poisson","yield_n_mm2",
        "plastic_hardening_n_mm2","rate_c_per_s","rate_p","rate_filter_hz","law"});
    Require(Text(Field(m,"law"))=="law44_linear"&&Text(Field(raw,"law"))=="law44_linear","Unsupported scene material law");
    for(const char* key:{"density_tonne_mm3","young_n_mm2","poisson","yield_n_mm2","plastic_hardening_n_mm2","rate_c_per_s","rate_p","rate_filter_hz"})SameNumber(Field(m,key),Field(raw,key));
    out.material={RealField(m,"density_tonne_mm3"),RealField(m,"young_n_mm2"),RealField(m,"poisson"),RealField(m,"yield_n_mm2"),
        RealField(m,"plastic_hardening_n_mm2"),RealField(m,"rate_c_per_s"),RealField(m,"rate_p"),RealField(m,"rate_filter_hz")};
    const auto& a=out.material;
    Require(a.density_tonne_mm3>=1e-15&&a.density_tonne_mm3<=1e-3&&a.young_n_mm2>=1e-6&&a.young_n_mm2<=1e9&&
        a.poisson>=0&&a.poisson<=.499&&a.yield_n_mm2>=1e-6&&a.yield_n_mm2<=a.young_n_mm2&&
        a.plastic_hardening_n_mm2>=0&&a.plastic_hardening_n_mm2<=a.young_n_mm2&&a.rate_c_per_s>=1e-9&&a.rate_c_per_s<=1e12&&
        a.rate_p>=.1&&a.rate_p<=100&&a.rate_filter_hz>=1e-9&&a.rate_filter_hz<=1e12,"Declared material exceeds supported scene range");
}
void Controls(DeclaredData& out,const Value& scene,const Value& original) {
    out.thickness_mm=RealField(scene,"thickness_mm");SameNumber(Field(scene,"thickness_mm"),Field(original,"thickness_mm"));
    const auto& run=Field(original,"run");Require(run.IsObject(),"Scene run controls must be an object");
    if(run.HasMember("time_step_cap_s"))Keys(run,{"end_time_s","nodal_scale","animation_interval_s","time_step_cap_s"});
    else Keys(run,{"end_time_s","nodal_scale","animation_interval_s"});
    out.end_time_s=RealField(scene,"end_time_s");out.nodal_scale=RealField(scene,"nodal_scale");
    out.animation_interval_s=RealField(scene,"animation_interval_s");
    for(const char* key:{"end_time_s","nodal_scale","animation_interval_s"})SameNumber(Field(scene,key),Field(run,key));
    const auto& cap=Field(scene,"time_step_cap_s");
    if(cap.IsNull())Require(!run.HasMember("time_step_cap_s"),"Missing exported timestep cap");
    else {out.step_cap_s=Real(cap);SameNumber(cap,Field(run,"time_step_cap_s"));}
    out.velocity_mm_s=Vector(Field(scene,"velocity_mm_s"));
    const auto raw_v=Vector(Field(Field(original,"patch"),"velocity_mm_s"));
    Require(Bits(raw_v.x)==Bits(out.velocity_mm_s.x)&&Bits(raw_v.y)==Bits(out.velocity_mm_s.y)&&Bits(raw_v.z)==Bits(out.velocity_mm_s.z),"Exported velocity differs from declared patch velocity");
    Require(out.thickness_mm>=1e-6&&out.thickness_mm<=100&&out.end_time_s>=1e-9&&out.end_time_s<=.1&&
        out.nodal_scale>=.01&&out.nodal_scale<=.9&&out.animation_interval_s>=1e-9&&out.animation_interval_s<=out.end_time_s&&
        out.end_time_s/out.animation_interval_s<=10000&&(!out.step_cap_s||(*out.step_cap_s>=1e-12&&*out.step_cap_s<=out.end_time_s))&&
        out.velocity_mm_s.z<0&&std::abs(out.velocity_mm_s.x)<=100000&&std::abs(out.velocity_mm_s.y)<=100000&&std::abs(out.velocity_mm_s.z)<=100000,
        "Unsupported declared scene motion/time/thickness");
}
void Mesh(DeclaredData& out,const Value& mesh,ReadLimits limits) {
    Keys(mesh,{"nodes","wall","patch","wall_nodes","patch_nodes"});
    const auto& nodes=Field(mesh,"nodes");Require(nodes.IsArray()&&nodes.Size()>=7&&nodes.Size()<=limits.nodes,"Scene node extent exceeds cap");
    std::map<std::uint64_t,std::uint32_t> index;
    for(const auto& node:nodes.GetArray()) {
        Keys(node,{"id","xyz_mm"});const auto id=Id(Field(node,"id"));
        Require(index.emplace(id,std::uint32_t(out.nodes.size())).second,"Duplicate declared node ID");
        out.nodes.push_back({id,Vector(Field(node,"xyz_mm"))});
    }
    std::vector<unsigned> membership(out.nodes.size(),0);std::set<std::uint64_t> element_ids;
    const auto group=[&](const char* name,unsigned role,std::vector<std::uint32_t>& output) {
        const auto& values=Field(mesh,name);Require(values.IsArray()&&values.Size()>=3&&values.Size()<=out.nodes.size(),"Scene node group exceeds extent");
        for(const auto& value:values.GetArray()) {
            const auto found=index.find(Id(value));Require(found!=index.end()&&!membership[found->second],"Scene groups overlap, duplicate, or omit source nodes");
            membership[found->second]=role;output.push_back(found->second);
        }
    };
    group("wall_nodes",1,out.wall_nodes);group("patch_nodes",2,out.patch_nodes);
    for(auto role:membership)Require(role!=0,"Declared scene contains an unassigned physical node");
    const auto parents=[&](const char* name,unsigned corners,unsigned role,std::vector<SourceParent>& output) {
        const auto& values=Field(mesh,name);Require(values.IsArray()&&values.Size()>0&&values.Size()<=limits.parents-element_ids.size(),"Scene parent extent exceeds cap");
        for(const auto& value:values.GetArray()) {
            Keys(value,{"id","part","nodes"});SourceParent parent;parent.id=Id(Field(value,"id"));parent.part=Id(Field(value,"part"));parent.corners=corners;
            Require(parent.part==role&&element_ids.insert(parent.id).second,"Scene parent identity/part differs");
            const auto& connectivity=Field(value,"nodes");Require(connectivity.IsArray()&&connectivity.Size()==corners,"Scene parent topology differs");
            for(unsigned k=0;k<corners;++k) {
                const auto found=index.find(Id(connectivity[k]));Require(found!=index.end()&&membership[found->second]==role,"Scene face uses a foreign node");
                parent.nodes[k]=found->second;
                for(unsigned j=0;j<k;++j)Require(parent.nodes[j]!=parent.nodes[k],"Repeated physical face node");
            }
            if(corners==3)parent.nodes[3]=parent.nodes[2];output.push_back(parent);
        }
    };
    parents("wall",3,1,out.wall);parents("patch",4,2,out.patch);
    Require(out.wall.size()>=4,"At least four authentic primary wall faces required");
    std::vector<unsigned> referenced(out.nodes.size());
    for(const auto* family:{&out.wall,&out.patch})for(const auto& parent:*family)
        for(unsigned k=0;k<parent.corners;++k)++referenced[parent.nodes[k]];
    for(auto count:referenced)Require(count>0,"Scene node lacks a physical shell contribution");
}
}
DeclaredSource DeclaredSource::Read(const std::filesystem::path& path,const std::string& expected,ReadLimits limits) {
    using namespace output;using namespace output::array_json;
    Require(limits.file_bytes&&limits.file_bytes<=4u<<20&&limits.nodes&&limits.nodes<=4096&&limits.parents&&limits.parents<=4096,"Declared scene limits exceed fixed source bounds");
    const auto bytes=ReadBounded(path,limits.file_bytes);Require(Sha256(bytes)==expected,"Declared scene export hash differs");
    const auto doc=Parse(bytes,limits.file_bytes);Keys(doc,{"schema","scope","source_sha256","scene","mesh","files"});
    Require(Text(Field(doc,"schema"))=="robo_dyna.native_contact_scene_export.v1","Unsupported declared scene export");
    (void)Text(Field(doc,"scope")); // Descriptive text is never numerical authority.
    auto out=std::make_shared<DeclaredData>();out->export_sha256=expected;out->definition_sha256=Text(Field(doc,"source_sha256"));
    const auto& files=Field(doc,"files");Require(files.IsArray()&&files.Size()==3,"Declared scene requires exact source/reference file inventory");
    std::set<std::string> seen;
    for(const auto& row:files.GetArray()) {
        Keys(row,{"path","bytes","sha256"});const auto name=Text(Field(row,"path"));
        Require((name=="scene.json"||name=="contact_scene_0000.rad"||name=="contact_scene_0001.rad")&&seen.insert(name).second,"Unexpected or duplicated scene file path");
        const auto size=UInt(Field(row,"bytes"));Require(size<=limits.file_bytes,"Scene file exceeds cap");
        const auto content=ReadBounded(path.parent_path()/name,limits.file_bytes);
        Require(content.size()==size&&Sha256(content)==Text(Field(row,"sha256")),"Declared source/reference bytes changed");
        if(name=="scene.json")out->definition_bytes=content;
    }
    Require(Sha256(out->definition_bytes)==out->definition_sha256,"Original scene member differs from source identity");
    const auto original=Parse(out->definition_bytes,limits.file_bytes);Keys(original,{"schema","units","wall","patch","material","thickness_mm","run"});
    Require(Text(Field(original,"schema"))=="robo_dyna.native_contact_scene.v1","Unknown original scene declaration");
    const auto& units=Field(original,"units");Keys(units,{"length","mass","time"});
    Require(Text(Field(units,"length"))=="mm"&&Text(Field(units,"mass"))=="tonne"&&Text(Field(units,"time"))=="s","Scene requires explicit native mm/tonne/s");
    const auto& scene=Field(doc,"scene");Keys(scene,{"wall","patch","velocity_mm_s","material","thickness_mm","end_time_s","nodal_scale","animation_interval_s","time_step_cap_s"});
    Materials(*out,scene,original);Controls(*out,scene,original);Mesh(*out,Field(doc,"mesh"),limits);
    return DeclaredSource(std::move(out));
}
} // namespace crash::modelio::native_scene
