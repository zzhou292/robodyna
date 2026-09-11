#include "Storage.h"
#include "Packing.h"
#include "Reports.h"
#include <cstring>
namespace crash::cases::vehicle_runtime {
namespace {
bool Bits(double a,double b) noexcept { return std::memcmp(&a,&b,sizeof(double))==0; }
template<class T> void ReadDevice(std::vector<T>& out,const T* data,cudaStream_t stream) {
    output::Require(data && cudaMemcpyAsync(out.data(),data,out.size()*sizeof(T),
        cudaMemcpyDeviceToHost,stream)==cudaSuccess,"Initial owner device readback failed");
}
}
void VehiclePhysicalStartup::Storage::InspectOwner(InitialInspection& out) {
    // Reuse one complete packing payload for all channels; no simultaneous
    // second full nodal snapshot. Immutable source values remain the oracle.
    auto values = detail::PackOwner(execution.model().coefficients(),execution.model().rigid_assembly(),
        roles,{InitialSpeedMps,0,0},forecast.packing_bytes);
    const auto n = roles.node.size();
    tl::fea::NodalStamp stamp;
    detail::RequireSuccess(owner.CopyAccepted({values.position.data(),values.velocity.data(),n,
        values.orientation.data(),values.spin.data()},&stamp));
    output::Require(stamp.owner_id==out.stamp.owner_id && stamp.epoch==0 && stamp.time==0,
                    "Initial nodal readback stamp changed");
    const auto& ledger = execution.model().coefficients();
    for (std::size_t i=0;i<n;++i) {
        const auto x = ledger.domain()->nodes()[i].position;
        output::Require(Bits(values.position[3*i],x.x) && Bits(values.position[3*i+1],x.y) &&
            Bits(values.position[3*i+2],x.z),"Initial node coordinates differ from canonical physical source");
        output::Require(Bits(values.velocity[3*i],InitialSpeedMps) && values.velocity[3*i+1]==0 &&
            values.velocity[3*i+2]==0,"Initial velocity differs from explicit 35 mph +X");
        output::Require(values.orientation[4*i]==1 && values.orientation[4*i+1]==0 &&
            values.orientation[4*i+2]==0 && values.orientation[4*i+3]==0 &&
            values.spin[3*i]==0 && values.spin[3*i+1]==0 && values.spin[3*i+2]==0,
            "Initial rotation differs from identity and zero spin");
    }
    tl::fea::NodalTrialToken token;
    tl::fea::NodalAssemblyView assembly;
    detail::RequireSuccess(owner.BeginTrial(&token,&assembly));
    try {
        ReadDevice(values.inverse_mass,assembly.mass.inverse_mass,assembly.stream);
        ReadDevice(values.inverse_inertia,assembly.inverse_inertia,assembly.stream);
        ReadDevice(values.fixed,assembly.translation_fixed_bits,assembly.stream);
        ReadDevice(values.rotation_fixed,assembly.rotation_fixed,assembly.stream);
        ReadDevice(values.rotation_present,assembly.rotation_present,assembly.stream);
        output::Require(cudaStreamSynchronize(assembly.stream)==cudaSuccess,"Initial DOF readback synchronization failed");
    } catch (...) {
        cudaStreamSynchronize(assembly.stream);
        owner.Discard();
        throw;
    }
    owner.Discard();
    for (std::size_t i=0;i<n;++i) {
        const auto c = ledger.nodes()[i].coefficients;
        const bool dependent = roles.node[i]&CinSecondary;
        const bool present = roles.node[i]!=0;
        const double inverse_mass = dependent || c.mass==0 ? 0 : 1/c.mass;
        const double inverse_inertia = dependent || !present || c.isotropic_inertia==0 ? 0 : 1/c.isotropic_inertia;
        output::Require(Bits(values.inverse_mass[i],inverse_mass) && Bits(values.inverse_inertia[i],inverse_inertia) &&
            values.fixed[i]==0 && values.rotation_fixed[i]==0 && values.rotation_present[i]==present,
            "Initial inverse coefficients or explicit DOF roles differ");
        out.absent_rotations += !present;
        out.cin_secondaries += dependent;
    }
    const auto rows = attachments.attachments().model().rows();
    output::Require(rows.count<=values.velocity.size(),"CIN snapshot exceeds reusable readback capacity");
    // The velocity/spin channels have already passed and now hold SMAS/SINER.
    detail::RequireSuccess(owner.CopyAcceptedCin({values.mass.data(),values.inertia.data(),
        values.velocity.data(),values.spin.data(),values.inverse_mass.data(),n,rows.count},&stamp));
    for (std::size_t i=0;i<n;++i) {
        const auto c = ledger.nodes()[i].coefficients;
        output::Require(Bits(values.mass[i],c.mass) && Bits(values.inertia[i],c.isotropic_inertia),
                        "Initial raw M/J differ from the single complete ledger");
    }
    for (std::size_t r=0;r<rows.count;++r) {
        const auto c = ledger.nodes()[rows.data[r].secondary_domain_node].coefficients;
        output::Require(Bits(values.velocity[r],c.mass) && Bits(values.spin[r],c.isotropic_inertia),
                        "Initial CIN saved secondary coefficients differ from native startup");
    }
    output::Require(values.inverse_mass[0]==0,"Initial CIN numerical mass is nonzero");
    out.nodes = n;
}
} // namespace crash::cases::vehicle_runtime
