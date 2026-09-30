#include "ComparisonInput.h"
#include "Metrics.h"

namespace crash::benchmarks::assembly_pilot {
namespace {
Metric& Channel(Metrics& m,const std::string& name,const char* units) {
    auto& out=m[name];out.units=units;return out;
}
double N(const Value& v){return rd::AssemblyNumber(v);}
std::uint64_t Id(const Value& v){return rd::AssemblyId(v);}
void Geometry(Metrics& m,const Value& a,const Value& b) {
    const auto& av=rd::Member(a,"nodal_fields");const auto& bv=rd::Member(b,"nodal_fields");
    Require(rd::Text(av,"position_phase")=="accepted_endpoint"&&rd::Text(bv,"position_phase")=="accepted_endpoint",
            "Pilot geometry is not an accepted endpoint");
    const auto& ids=rd::Member(rd::Member(a,"sections"),"vertex_binding");
    Require(ids==rd::Member(rd::Member(b,"sections"),"vertex_binding"),"Pilot nodal source binding differs");
    Require(ids.IsArray()&&ids.Size()>0&&ids.Size()<=4096,"Pilot node extent invalid");const auto count=ids.Size();
    const auto& x=rd::WallNumbers(av,"position_xyz_m",3*count);const auto& y=rd::WallNumbers(bv,"position_xyz_m",3*count);
    const auto& q=rd::WallNumbers(av,"orientation_wxyz",4*count);const auto& r=rd::WallNumbers(bv,"orientation_wxyz",4*count);
    for(std::size_t n=0;n<count;++n) {
        const auto source=Id(rd::AssemblyRow(ids[n],4)[3]);long double delta=0,reference=0;
        for(unsigned k=0;k<3;++k) {const long double d=static_cast<long double>(N(x[3*n+k]))-N(y[3*n+k]);
            delta+=d*d;reference+=static_cast<long double>(N(x[3*n+k]))*N(x[3*n+k]);}
        Channel(m,"node_position_distance","m").Distance(std::sqrt(reference),std::sqrt(delta),n,source);
        std::array<double,4> qa{},qb{};for(unsigned k=0;k<4;++k){qa[k]=N(q[4*n+k]);qb[k]=N(r[4*n+k]);}
        Channel(m,"node_orientation_distance","rad").Distance(RotationDistance({1,0,0,0},qa),RotationDistance(qa,qb),n,source);
    }
}
void Sections(Metrics& m,const Value& a,const Value& b) {
    Require(rd::Text(a,"stress_frame")=="native_corotational_shell_axes"&&rd::Text(a,"stress_frame")==rd::Text(b,"stress_frame"),
            "Pilot stress frame changed");
    const auto& sa=rd::Member(a,"sections");const auto& sb=rd::Member(b,"sections");
    for(const char* key:{"parent_columns","section_columns","point_columns","section_position_over_thickness",
                        "section_force_weight","section_moment_weight","source_parents"})
        Require(rd::Member(sa,key)==rd::Member(sb,key),"Pilot native parent/layer source association changed");
    const auto& parents=rd::Member(sa,"source_parents");Require(parents.IsArray()&&parents.Size()>0&&parents.Size()<=1024,"Pilot parent extent invalid");
    const auto& x=rd::WallArray(sa,"sections",parents.Size());const auto& y=rd::WallArray(sb,"sections",parents.Size());
    constexpr const char* stress[]{"layer_native_stress_XX","layer_native_stress_YY","layer_native_stress_XY",
                                 "layer_native_stress_YZ","layer_native_stress_ZX"};
    for(std::size_t p=0;p<parents.Size();++p) {
        const auto source=Id(rd::AssemblyRow(parents[p],11)[1]);rd::AssemblyRow(x[p],10);rd::AssemblyRow(y[p],10);
        Channel(m,"parent_cumulative_plastic_work","J").Add(N(x[p][0]),N(y[p][0]),p,source);
        Channel(m,"parent_reported_thickness","m").Add(N(x[p][8]),N(y[p][8]),p,source);
        const auto& xp=rd::AssemblyRow(x[p][9],3);const auto& yp=rd::AssemblyRow(y[p][9],3);
        for(unsigned layer=0;layer<3;++layer) {
            rd::AssemblyNumbers(xp[layer],7);rd::AssemblyNumbers(yp[layer],7);const auto i=3*p+layer;
            for(unsigned k=0;k<5;++k)Channel(m,stress[k],"Pa").Add(N(xp[layer][k]),N(yp[layer][k]),i,source);
            Channel(m,"layer_equivalent_plastic_strain","1").Add(N(xp[layer][5]),N(yp[layer][5]),i,source);
        }
    }
}
void Scalars(Metrics& m,const Value& a,const Value& b) {
    const auto& x=rd::Member(a,"diagnostics");const auto& y=rd::Member(b,"diagnostics");
    for(const char* key:{"native_internal_work_J","cumulative_plastic_work_J"})Channel(m,key,"J").Add(rd::Real(x,key),rd::Real(y,key));
    Channel(m,"maximum_plastic_strain","1").Add(rd::Real(x,"maximum_plastic_strain"),rd::Real(y,"maximum_plastic_strain"));
    for(const char* key:{"yielded_points","yielded_parents","active_contact_nodes"})Channel(m,key,"count").Add(rd::Unsigned(x,key),rd::Unsigned(y,key));
    for(const char* family:{"qeph","t3"}) {
        const auto& fx=rd::Member(rd::Member(x,"shells"),family);const auto& fy=rd::Member(rd::Member(y,"shells"),family);
        const auto& wx=rd::WallNumbers(fx,"native_internal_work_J",2);const auto& wy=rd::WallNumbers(fy,"native_internal_work_J",2);
        for(unsigned k=0;k<2;++k)Channel(m,std::string(family)+"_native_work_"+std::to_string(k),"J").Add(N(wx[k]),N(wy[k]));
        if(std::string(family)=="qeph")Channel(m,"qeph_hourglass_viscous_work","J").Add(rd::Real(fx,"hourglass_viscous_work_J"),rd::Real(fy,"hourglass_viscous_work_J"));
    }
}
void Certificate(Metrics& m,const char* name,const char* units,const Value& a,const Value& b,std::size_t index=0,std::uint64_t source=0) {
    const auto x=rd::WallCertificate(a),y=rd::WallCertificate(b);
    Channel(m,name,units).Add(x[0],y[0],index,source);
    const auto gap=std::max({0.L,static_cast<long double>(x[1])-y[2],static_cast<long double>(y[1])-x[2]});
    Channel(m,std::string(name)+"_certificate_gap",units).Distance(0,gap,index,source);
}
void Contact(Metrics& m,const Value& a,const Value& b) {
    const auto& x=rd::Member(a,"contact");const auto& y=rd::Member(b,"contact");
    if(x.IsNull()||y.IsNull()){Require(x.IsNull()&&y.IsNull(),"Pilot initial contact phases differ");return;}
    Require(rd::Text(x,"phase")=="prepared_candidate_of_accepted_interval"&&rd::Text(x,"phase")==rd::Text(y,"phase"),"Pilot contact phase changed");
    Certificate(m,"wall_resultant","N",rd::Member(x,"resultant_N"),rd::Member(y,"resultant_N"));
    Certificate(m,"wall_potential","J",rd::Member(x,"potential_J"),rd::Member(y,"potential_J"));
    Channel(m,"maximum_penetration","m").Add(rd::Real(x,"maximum_penetration_m"),rd::Real(y,"maximum_penetration_m"));
    for(const auto& pair:{std::pair<const char*,const char*>{"wall_reaction_xyz_N","N"},{"wall_moment_xyz_N_m","N m"}}) {
        const auto& av=rd::WallNumbers(x,pair.first,3);const auto& bv=rd::WallNumbers(y,pair.first,3);
        for(unsigned k=0;k<3;++k)Channel(m,pair.first,pair.second).Add(N(av[k]),N(bv[k]),k);
    }
    Require(rd::Text(x,"node_columns")==rd::Text(y,"node_columns"),"Pilot contact node columns changed");
    const auto& nodes=rd::Member(x,"nodes");Require(nodes.IsArray()&&nodes.Size()>0&&nodes.Size()<=4096,"Pilot contact extent invalid");
    const auto& other=rd::WallArray(y,"nodes",nodes.Size());
    for(std::size_t n=0;n<nodes.Size();++n) {
        const auto& u=rd::AssemblyRow(nodes[n],13);const auto& v=rd::AssemblyRow(other[n],13);
        Require(Id(u[0])==n&&Id(v[0])==n&&Id(u[1])==Id(v[1]),"Pilot contact source node changed");
        Certificate(m,"node_contact_force","N",u[3],v[3],n,Id(u[1]));Certificate(m,"node_contact_potential","J",u[4],v[4],n,Id(u[1]));
        Channel(m,"node_contact_active_changed","indicator").Add(N(u[3][0])>0,N(v[3][0])>0,n,Id(u[1]));
        Channel(m,"node_wall_triangle_changed","indicator").Add(0,Id(u[2])!=Id(v[2]),n,Id(u[1]));
    }
}
}
Metrics Difference(const Value& a,const Value& b) {
    Metrics out;Geometry(out,a,b);Sections(out,a,b);Scalars(out,a,b);Contact(out,a,b);return out;
}
} // namespace crash::benchmarks::assembly_pilot
