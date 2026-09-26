#include "CoupledDeclaration.h"
#include <algorithm>
#include <set>
namespace crash::modelio::native_scene::detail {
namespace {
using namespace output;using namespace output::array_json;
const Value& Get(const Value& v,const char* key){Require(v.IsObject()&&v.HasMember(key),"Missing coupled scene field");return v[key];}
void Exact(const Value& v,double expected){Require(Bits(Real(v))==Bits(expected),"Coupled reference source scalar differs");}
void Vector(const Value& v,tl::math::Vec3 expected){Require(v.IsArray()&&v.Size()==3,"Coupled reference vector extent differs");Exact(v[0],expected.x);Exact(v[1],expected.y);Exact(v[2],expected.z);}
void Identifiers(const Value& v,const std::vector<std::uint64_t>& expected){Require(v.IsArray()&&v.Size()==expected.size(),"Coupled reference roster extent differs");for(std::size_t i=0;i<expected.size();++i)Require(UInt(v[unsigned(i)])==expected[i],"Coupled reference node order differs");}
}
void BindRigidPatch(DeclaredData& out,const Value& exported,const Value& original) {
    using namespace output;using namespace output::array_json;
    for(const auto* coupling:{&Get(Get(exported,"scene"),"coupling"),&Get(original,"coupling")}) {
        Keys(*coupling,{"kind","primary_initialization","inertia_mode","center_of_gravity","tied_secondary_removal","primary_velocity"});
        Require(Text(Get(*coupling,"kind"))=="rigid_patch"&&Text(Get(*coupling,"primary_initialization"))=="converted_part"&&
            UInt(Get(*coupling,"inertia_mode"))==2&&UInt(Get(*coupling,"center_of_gravity"))==1&&
            UInt(Get(*coupling,"tied_secondary_removal"))==1&&Text(Get(*coupling,"primary_velocity"))=="patch_translation",
            "Unsupported coupled scene source controls");
    }
    Require(out.contact_surface==DeclaredContactSurface::AllShells,"Rigid scene requires all declared primary shells");
    DeclaredRigidPatch next;next.source_part_id=2;next.reference_body_id=1;
    std::uint64_t maximum=0;for(const auto& n:out.nodes)maximum=std::max(maximum,n.id);
    Require(maximum<UINT32_MAX,"Generated reference primary ID exceeds native domain");next.reference_primary_id=maximum+1;
    for(auto n:out.patch_nodes)next.member_source_ids.push_back(out.nodes[n].id);
    std::sort(next.member_source_ids.begin(),next.member_source_ids.end());
    std::set<std::uint64_t> encountered;
    for(const auto& p:out.patch)for(unsigned k=0;k<p.corners;++k) {
        const auto& n=out.nodes[p.nodes[k]];
        if(encountered.insert(n.id).second) {
            next.centroid_source_order.push_back(n.id);
            next.reference_primary_mm.x+=n.xyz_mm.x;next.reference_primary_mm.y+=n.xyz_mm.y;next.reference_primary_mm.z+=n.xyz_mm.z;
        }
    }
    Require(encountered==std::set<std::uint64_t>(next.member_source_ids.begin(),next.member_source_ids.end()),"Rigid source membership differs from physical patch coverage");
    const double count=double(next.centroid_source_order.size());Require(count>=3,"Rigid patch lacks physical members");
    next.reference_primary_mm.x/=count;next.reference_primary_mm.y/=count;next.reference_primary_mm.z/=count;
    const auto& body=Get(exported,"reference_rigid_body");
    Keys(body,{"body_id","source_part_id","primary","centroid_node_order","member_node_ids","converter_mass_tonne",
        "converter_inertia_tonne_mm2","off_diagonal_inertia_tonne_mm2","inertia_mode","center_of_gravity","primary_velocity_mm_s"});
    Require(UInt(Get(body,"body_id"))==next.reference_body_id&&UInt(Get(body,"source_part_id"))==next.source_part_id&&
        UInt(Get(body,"inertia_mode"))==2&&UInt(Get(body,"center_of_gravity"))==1,"Exported rigid group controls differ");
    const auto& primary=Get(body,"primary");Keys(primary,{"id","xyz_mm"});
    Require(UInt(Get(primary,"id"))==next.reference_primary_id,"Reference primary is not the declared generated source ID");
    Vector(Get(primary,"xyz_mm"),next.reference_primary_mm);Identifiers(Get(body,"centroid_node_order"),next.centroid_source_order);
    Identifiers(Get(body,"member_node_ids"),next.member_source_ids);Exact(Get(body,"converter_mass_tonne"),1e-20);
    Vector(Get(body,"converter_inertia_tonne_mm2"),{1e-20,1e-20,1e-20});Vector(Get(body,"off_diagonal_inertia_tonne_mm2"),{});
    Vector(Get(body,"primary_velocity_mm_s"),out.velocity_mm_s);
    out.rigid_patch=std::move(next);
}
}
