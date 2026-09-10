#include "AnalysisInternal.h"
#include <algorithm>
#include <cmath>

namespace crash::benchmarks::assembly_spin {
const Value& Numbers(const Value& v,const char* key,std::size_t n) {
    const auto& a=json::Array(v,key,n,n);for(const auto& x:a.GetArray())json::Real(x);return a;
}
Document Parse(const char* bytes,std::size_t count) {
    Require(count&&count<RowCap,"Spin JSON row exceeds its bound");Document d;
    d.Parse<rapidjson::kParseFullPrecisionFlag>(bytes,count);Require(!d.HasParseError()&&d.IsObject(),"Spin JSON row is malformed");
    json::UniqueKeys(d);return d;
}
Context::Context(const std::filesystem::path& path):inventory(source::SourceAssembly::Read(path,source::PinnedYarisSixPartInventory())),
    materials(inventory,source::MaterialRatePolicy::OpenRadiossDirectImportDefault) {}
void Context::Header(const Value& h) {
    json::TextIs(h,"schema","robo_dyna.source_assembly_qeph_spin_trace.v1");json::TextIs(h,"record","header");
    const auto& data=inventory.data();Require(json::Text(h,"source_inventory_sha256")==data.identity.sha256&&
        json::Unsigned(h,"source_inventory_bytes")==data.identity.bytes,"Spin header source artifact differs");
    source_node=json::Unsigned(h,"source_node_id");source_instance=json::Unsigned(h,"source_instance_id");owner=json::Unsigned(h,"owner_id");
    Require(source_node&&source_instance&&owner&&json::Unsigned(h,"configuration_id")&&json::Unsigned(h,"qualification_id")&&
        json::Unsigned(h,"wall_binding_id"),"Spin header identity is incomplete");
    requested=json::Unsigned(h,"requested_steps",1u<<20);cadence=json::Unsigned(h,"cadence",requested);dt=json::Real(h,"fixed_dt_s");
    Require(requested&&cadence&&dt>0&&std::isfinite(requested*dt),"Spin request has invalid steps/time");
    Require(json::Unsigned(h,"row_byte_cap")==RowCap&&json::Unsigned(h,"total_byte_cap")==TraceCap&&
        4+requested/cadence<=TraceCap/RowCap&&json::Unsigned(h,"forecast_bytes")==RowCap*(4+requested/cadence),"Spin forecast scope changed");
    global_node=json::NodeIndex(data,source_node);
    for(const auto& g:data.nodal_rigid_groups)if(g.internal) {
        ++group_count;member_count+=g.members.size();
        Require(std::find(g.members.begin(),g.members.end(),source_node)==g.members.end(),"Spin source node belongs to a rigid group");
    }
    const source::SourceAssemblyShellInput shell_input(inventory);const auto shells=shell_input.input();const auto mat=materials.input();
    for(const auto& p:data.parents)for(unsigned local=0;local<p.arity;++local)if(p.nodes[local]==global_node) {
        Require(p.family==source::ShellFamily::Qeph&&parents.size()<8,"Spin source omits a T3 or excess incident parent");
        ParentSource next;next.parent=&p;next.local=local;const auto& input=shells.qeph[p.family_index].reference;
        native::ReferenceInput original;for(unsigned i=0;i<4;++i){original.position[i]=input.position[i];original.node_ids[i]=input.node_ids[i];}
        original.density=input.density;original.young_modulus=input.young_modulus;original.poisson_ratio=input.poisson_ratio;original.thickness=input.thickness;
        Require(native::Initialize(original,next.reference)==native::Status::kSuccess,"Original source native QEPH reference failed");
        const auto& curve=data.curves[p.curve_index];const auto& rate=mat.materials[p.material_index].rate;
        Require(rate.enabled,"Spin source material rate policy differs");
        next.law={curve.plastic_strain.data(),curve.stress_pa.data(),curve.plastic_strain.size(),
                  rate.cowper_symonds_c_per_s,rate.cowper_symonds_p,rate.cutoff_hz};
        parents.push_back(next);
    }
    Require(!parents.empty(),"Spin source node has no parents");
}
void Context::Stamp(const Value& s,std::uint64_t epoch) const {
    Require(json::Unsigned(s,"owner_id")==owner&&json::Unsigned(s,"epoch")==epoch&&
        json::Unsigned(s,"node_count")==inventory.data().nodes.size(),"Spin stamp owner/epoch/count differs");
    json::TextIs(s,"temporal_scheme","staggered_half_kick_start");json::TextIs(s,"velocity_phase",epoch?"previous_midpoint":"collocated");
    json::Flag(s,"has_rotations",true);json::Flag(s,"reactions_valid",epoch!=0);json::Same(json::Real(s,"fixed_dt"),dt);
    const auto& g=json::Member(s,"rigid_groups");Require(json::Unsigned(g,"source_instance_id")==source_instance&&
        json::Unsigned(g,"group_count")==group_count&&json::Unsigned(g,"member_count")==member_count,"Spin group scope differs");
    const double t=json::Real(s,"time"),v=json::Real(s,"velocity_time"),r=json::Real(s,"reaction_time");
    Require(json::Unsigned(s,"reaction_base_epoch")== (epoch?epoch-1:0),"Spin reaction epoch differs");
    if(!epoch) {json::Same(t,0);json::Same(v,0);json::Same(r,0);json::Same(json::Real(s,"reaction_kick_dt"),0);}
    else {
        Require(r>=0&&t==r+dt&&v==r+.5*dt&&t>v&&v>r,"Spin stored phase is not the accepted staggered endpoint");
        json::Same(json::Real(s,"reaction_kick_dt"),epoch==1?.5*dt:dt);
        // Preserve accumulated binary64 clocks; validate their bounded relation
        // to the nominal grid, without claiming carried velocity is collocated.
        const long double nominal=static_cast<long double>(epoch)*dt;
        Require(std::abs(static_cast<long double>(t)-nominal)<=8*std::numeric_limits<double>::epsilon()*epoch*nominal,
                "Spin stamp is outside the declared fixed-step grid");
    }
}
}
