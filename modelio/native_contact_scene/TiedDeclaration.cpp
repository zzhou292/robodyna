#include "TiedDeclaration.h"
#include <cmath>
namespace crash::modelio::native_scene::detail {
namespace {
using namespace output;using namespace output::array_json;
const Value& Get(const Value& v,const char* name){Require(v.IsObject()&&v.HasMember(name),"Missing tied source field");return v[name];}
void Exact(const Value& v,double expected){Require(Bits(Real(v))==Bits(expected),"Tied source scalar differs");}
struct Grid {std::vector<double> x,y;double z=0,slope=0;};
Grid ReadGrid(const Value& raw,const Value& exported,unsigned count,bool velocity=false) {
    if(velocity)Keys(raw,{"x_mm","y_mm","z_mm","dz_dx","velocity_mm_s"});
    else Keys(raw,{"x_mm","y_mm","z_mm","dz_dx"});
    Keys(exported,{"x_mm","y_mm","z_mm","dz_dx"});
    Grid grid;
    const auto axis=[&](const char* name,std::vector<double>& out) {
        const auto& a=Get(raw,name);const auto& b=Get(exported,name);
        Require(a.IsArray()&&b.IsArray()&&a.Size()==count&&b.Size()==count,"Named tied grid extent differs");
        for(unsigned i=0;i<count;++i) {
            const auto value=Real(a[i]);Exact(b[i],value);
            Require(std::isfinite(value)&&value>=-10000&&value<=10000&&(!i||value>out.back()),"Tied grid axis is invalid");out.push_back(value);
        }
    };
    axis("x_mm",grid.x);axis("y_mm",grid.y);
    grid.z=Real(Get(raw,"z_mm"));grid.slope=Real(Get(raw,"dz_dx"));
    Exact(Get(exported,"z_mm"),grid.z);Exact(Get(exported,"dz_dx"),grid.slope);
    Require(std::isfinite(grid.z)&&std::abs(grid.z)<=10000&&grid.slope==0,"Named tied surfaces must be flat");return grid;
}
void Positions(const DeclaredData& d,const Grid& grid,std::size_t first) {
    std::size_t index=first;
    for(double y:grid.y)for(double x:grid.x) {
        const auto& node=d.nodes.at(index);
        Require(node.id==index+1&&Bits(node.xyz_mm.x)==Bits(x)&&Bits(node.xyz_mm.y)==Bits(y)&&
            Bits(node.xyz_mm.z)==Bits(grid.z+grid.slope*x),"Physical node differs from the declared source grid");++index;
    }
}
void Controls(const Value& c,DeclaredTiedPatch& result) {
    Keys(c,{"kind","dependent","ignore","spotflag","level","search","deletion","search_distance_mm", "stiffness_scale","viscosity","stiffness_mode","tied_secondary_removal"});
    Require(Text(Get(c,"kind"))=="tied_patch"&&UInt(Get(c,"ignore"))==2&&UInt(Get(c,"spotflag"))==28&&
        UInt(Get(c,"level"))==0&&UInt(Get(c,"search"))==0&&UInt(Get(c,"deletion"))==1&&
        UInt(Get(c,"stiffness_mode"))==2&&UInt(Get(c,"tied_secondary_removal"))==1,"Unsupported TYPE2 source controls");
    Exact(Get(c,"search_distance_mm"),0.);Exact(Get(c,"stiffness_scale"),1.);Exact(Get(c,"viscosity"),.05);
    result.ignore=2;result.spotflag=28;result.level=0;result.search=0;result.deletion=1;
    result.stiffness_mode=2;result.tied_removal=1;result.search_distance_mm=0.;result.stiffness_scale=1.;result.viscosity=.05;
    // Original HM_READ_INTER_TYPE02: ILEV28 retained, Isearch0->2, hierarchy0.
    result.resolved_level=28;result.resolved_search=2;result.resolved_hierarchy=0;
}
void Parent(const SourceParent& p,std::uint64_t id,std::uint64_t part,std::array<std::uint32_t,4> nodes,unsigned corners) {
    Require(p.id==id&&p.part==part&&p.nodes==nodes&&p.corners==corners,"Named tied physical parent/order differs");
}
void Ids(const Value& a,const DeclaredData& d,const std::array<std::uint32_t,4>& rows) {
    Require(a.IsArray()&&a.Size()==4,"Tied reference node extent differs");
    for(unsigned k=0;k<4;++k)Require(UInt(a[k])==d.nodes.at(rows[k]).id,"Tied reference node/source order differs");
}
}
void BindTiedPatch(DeclaredData& out,const Value& exported,const Value& original) {
    using namespace output;using namespace output::array_json;
    Require(!out.rigid_patch&&out.contact_surface==DeclaredContactSurface::AllShells&&out.nodes.size()==17&&
        out.wall.size()==8&&out.patch.size()==2&&out.wall_nodes.size()==9&&out.patch_nodes.size()==8,
        "Version4 requires its complete17-node/10-parent ordinary shell source");
    const auto& scene=Get(exported,"scene");const auto& raw=Get(original,"coupling");const auto& resolved=Get(scene,"coupling");
    DeclaredTiedPatch next;Controls(raw,next);Controls(resolved,next);
    const auto wall=ReadGrid(Get(original,"wall"),Get(scene,"wall"),3);
    const auto master=ReadGrid(Get(original,"patch"),Get(scene,"patch"),2,true);
    const auto dependent=ReadGrid(Get(raw,"dependent"),Get(resolved,"dependent"),2);
    Require(master.z==dependent.z&&master.z>wall.z+out.thickness_mm&&master.x[0]<dependent.x[0]&&
        dependent.x[1]<master.x[1]&&master.y[0]<dependent.y[0]&&dependent.y[1]<master.y[1],
        "Tied dependent must be coplanar/interior and the physical patches initially wall-separated");
    Positions(out,wall,0);Positions(out,master,9);Positions(out,dependent,13);
    for(unsigned i=0;i<9;++i)Require(out.wall_nodes[i]==i,"Wall source group order differs");
    for(unsigned i=0;i<8;++i)Require(out.patch_nodes[i]==i+9,"Moving source group order differs");
    std::size_t parent=0;
    for(unsigned y=0;y<2;++y)for(unsigned x=0;x<2;++x) {
        const auto a=3*y+x,b=a+1,c=a+4,d=a+3;
        Parent(out.wall[parent],parent+1,1,{a,b,c,c},3);++parent;
        Parent(out.wall[parent],parent+1,1,{a,c,d,d},3);++parent;
    }
    Parent(out.patch[0],9,2,{9,10,12,11},4);Parent(out.patch[1],10,3,{13,14,16,15},4);
    next.interface_id=2;next.master_surface_id=2;next.secondary_group_id=3;
    next.master_parent_id=9;next.master_part_id=2;next.dependent_parent_id=10;next.dependent_part_id=3;
    next.master_nodes=out.patch[0].nodes;next.secondary_nodes={13,14,15,16};
    const auto& reference=Get(exported,"reference_tied_interface");
    Keys(reference,{"interface_id","master_surface_id","secondary_group_id","master_source_element_id","master_node_ids","secondary_node_ids", "ignore","spotflag","level","search","deletion","search_distance_mm","stiffness_scale","viscosity","stiffness_mode","tied_secondary_removal","scope"});
    Require(UInt(Get(reference,"interface_id"))==2&&UInt(Get(reference,"master_surface_id"))==2&&
        UInt(Get(reference,"secondary_group_id"))==3&&UInt(Get(reference,"master_source_element_id"))==9,
        "Exported TYPE2 source identity differs");
    Ids(Get(reference,"master_node_ids"),out,next.master_nodes);Ids(Get(reference,"secondary_node_ids"),out,next.secondary_nodes);
    for(const auto* name:{"ignore","spotflag","level","search","deletion","stiffness_mode","tied_secondary_removal"})
        Require(UInt(Get(reference,name))==UInt(Get(raw,name)),"Exported TYPE2 integer control differs");
    for(const auto* name:{"search_distance_mm","stiffness_scale","viscosity"})Exact(Get(reference,name),Real(Get(raw,name)));
    (void)Text(Get(reference,"scope"));out.tied_patch=next;
}
}
