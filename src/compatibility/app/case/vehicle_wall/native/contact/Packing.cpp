#include "Internal.h"
#include "lib_src/math/ScalarBits.h"
#include <algorithm>
#include <climits>
#include <numeric>
namespace crash::cases::vehicle_wall::native::wall_interface::detail {
std::size_t Fields::capacity_bytes() const noexcept {
    return ids.capacity()*sizeof(std::uint64_t)+(positions.capacity()+global_k.capacity()+secondary_k.capacity()+
        secondary_gap.capacity())*sizeof(double)+nodes.capacity()*sizeof(n::lifecycle::Node)+nsv.capacity()*sizeof(std::uint32_t);
}
s::Input Fields::Mesh(Declaration declaration,n::UnitScale units) const noexcept {
    s::Input input;
    input.profile=s::Profile::OrdinaryExteriorFixedMain;
    input.topology=s::TopologyPolicy::ManifoldTwoSided;
    input.node_source_ids=ids.data();input.node_count=ids.size();
    input.positions={positions.data(),std::uint32_t(ids.size()),3,1};
    input.primary=&primary;input.primary_count=1;
    input.coordinates=s::Coordinates::Native;input.units=units;
    input.source_generation=declaration.source_generation;
    return input;
}
void Allocate(Fields& out,const Plan& plan) {
    const auto n=plan.shape.nodes;
    out.ids.resize(n);out.positions.resize(3*n);out.nodes.resize(n);out.nsv.resize(n);
    out.global_k.resize(n);out.secondary_k.resize(n);out.secondary_gap.resize(n);
    if(out.capacity_bytes()+plan.topology.output_bytes+plan.topology.ready_output_bytes+(64u<<10)>plan.forecast.own_retained)
        Reject(Status::ResourceLimit,"Actual finite-wall retained vector capacity exceeds forecast");
}
void PackNodes(Fields& out,const EnvelopeOwnerSource& owner,const VehicleSource& vehicle,const Plan& plan) {
    const auto& model=owner.execution_source().mechanical();const auto& domain=model.domain();
    const auto& wall=model.wall();const auto source=vehicle.startup_input();
    const auto n=plan.shape.nodes,v=plan.shape.vehicle_nodes;
    if(source.coordinates!=s::Coordinates::Native)
        Reject(Status::UnsupportedProfile,"Finite-wall source requires authentic native prefix coordinates");
    // Reuse the qualified source packing to obtain final declared constraints;
    // the wall-only mask contribution is not treated as absence of other roles.
    const auto actual=owner.PackOwner(plan.forecast.constraint_packing);
    if(actual.fixed.size()!=n || actual.rotation_fixed.size()!=n)
        Reject(Status::SourceMismatch,"Actual owner constraint packing extent differs");
    for(std::size_t i=0;i<n;++i) {
        const auto id=domain.nodes()[i].source_id;
        if(!id || id>INT_MAX)Reject(Status::UnsupportedProfile,"Source NID exceeds pinned native integer range");
        out.ids[i]=id;
        if(actual.fixed[i]!=0 && actual.fixed[i]!=7)
            Reject(Status::UnsupportedProfile,"Wall interface declaration requires global free/fixed node subspaces");
        out.nodes[i]={id,int(actual.fixed[i]),actual.fixed[i]?1:0};
        if(i<v) {
            if(id!=source.node_source_ids[i])Reject(Status::SourceMismatch,"Original contact NID order differs from combined domain");
            const auto x=source.positions.at(std::uint32_t(i));
            out.positions[3*i]=x.x;out.positions[3*i+1]=x.y;out.positions[3*i+2]=x.z;
        } else {
            const auto x=wall.geometry().reference_native[i-v];
            out.positions[3*i]=x.x;out.positions[3*i+1]=x.y;out.positions[3*i+2]=x.z;
            if(actual.fixed[i]!=7 || actual.rotation_fixed[i]!=1)
                Reject(Status::SourceMismatch,"Actual appended wall node is not fully fixed");
        }
    }
    // Explicit ALL retained vehicle nodes, plus ILEV1's mandatory main-node
    // union. Physical domain indices do not change; I25SORS orders NSV by NID.
    std::iota(out.nsv.begin(),out.nsv.end(),std::uint32_t{0});
    std::sort(out.nsv.begin(),out.nsv.end(),[&](auto a,auto b){return out.ids[a]<out.ids[b];});
    for(std::size_t i=1;i<n;++i)if(out.ids[out.nsv[i-1]]>=out.ids[out.nsv[i]])
        Reject(Status::SourceMismatch,"Complete wall NSV contains duplicate source NIDs");
    out.msr=plan.shape.wall_nodes;
    // MSR retains I25SURFI first encounter in the one ordered primary; only NSV is sorted.
    out.primary.source_id=wall.ids().shell;out.primary.layout=n::ShellLayout::Quad4;
    for(unsigned k=0;k<4;++k)out.primary.nodes[k]=plan.shape.wall_nodes[k];
    const auto corrected=vehicle.gap_operands().corrected().coefficients();
    std::copy_n(corrected.data(),v,out.global_k.begin());
}
}
