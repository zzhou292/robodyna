#include "SourcePartElasticInternal.h"
#include <algorithm>
#include <cmath>

namespace crash::cases::source_part_elastic {
Report SourcePartElasticCase::Impl::Initialize(const source::SourcePartContactFixture& input,const Config& settings) {
    source=input; config=settings;
    const auto cr=collection.Initialize(source);
    if(cr.status!=source::FixtureStatus::Ok) return Failure(Status::ComponentFailure,"Original source collection conversion failed");
    const auto br=binding.Initialize(collection.input());
    if(br.status!=fe::ShellBindingStatus::Success) return Failure(Status::ComponentFailure,br.message);
    if(binding.node_count()!=NodeCount||binding.qeph_count()!=source::Q4Count||binding.t3_count()!=source::T3Count)
        return Failure(Status::ComponentFailure,"Incomplete original source collection");
    accepted.position=source.coordinates();
    double low=accepted.position[config.spatial_axis],high=low;
    for(std::size_t n=0;n<NodeCount;++n) {
        const auto& m=binding.nodes()[n].native;
        inverse_mass[n]=1/m.mass; inverse_inertia[n]=1/m.isotropic_inertia;
        accepted.orientation[4*n]=1;
        low=std::min(low,accepted.position[3*n+config.spatial_axis]);
        high=std::max(high,accepted.position[3*n+config.spatial_axis]);
    }
    if(!std::isfinite(high-low)||high<=low) return Failure(Status::InvalidInput,"Pulse spatial axis has no source extent");
    long double weighted=0,mass=0;
    const double middle=.5*(low+high),length=high-low;
    for(std::size_t n=0;n<NodeCount;++n) {
        const double xi=2*(accepted.position[3*n+config.spatial_axis]-middle)/length;
        spatial_shape[n]=xi*xi;
        weighted+=static_cast<long double>(binding.nodes()[n].native.mass)*spatial_shape[n];
        mass+=binding.nodes()[n].native.mass;
    }
    const double mean=static_cast<double>(weighted/mass);
    for(std::size_t n=0;n<NodeCount;++n) {
        spatial_shape[n]-=mean;
        for(unsigned a=0;a<3;++a) {
            pulse_force[3*n+a]=binding.nodes()[n].native.mass*config.acceleration*spatial_shape[n]*config.direction[a];
            if(!std::isfinite(pulse_force[3*n+a])) return Failure(Status::InvalidInput,"Pulse force construction overflow");
        }
    }
    fe::NodalStateConfig state;
    state.node_count=NodeCount; state.fixed_dt=config.dt;
    state.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    std::array<std::uint8_t,NodeCount> free{};
    const auto nr=owner.Initialize(state,{accepted.position.data(),accepted.velocity.data(),accepted.omega.data(),
        NodeCount,accepted.orientation.data()},inverse_mass.data(),{free.data(),free.data(),inverse_inertia.data()});
    if(nr.status!=fe::NodalStatus::Ok) return Failure(Status::ComponentFailure,nr.message);
    if(cudaMalloc(reinterpret_cast<void**>(&device_pulse),sizeof(pulse_force))!=cudaSuccess||
       cudaMemcpy(device_pulse,pulse_force.data(),sizeof(pulse_force),cudaMemcpyHostToDevice)!=cudaSuccess)
        return Failure(Status::DeviceFailure,"Pulse device setup failed");
    q::QephBatchConfig qc; qc.owner=owner.accepted(); qc.element_count=source::Q4Count;
    qc.configuration_id=config.configuration_id; qc.qualification_id=config.qualification_id;
    qc.usage=q::BatchUsage::CoupledForces;
    const auto qr=qeph.InitializeJoined(qc,binding);
    if(qr.status!=q::BatchStatus::Success) return Failure(Status::ComponentFailure,qr.message);
    t::T3BatchConfig tc; tc.owner=owner.accepted(); tc.element_count=source::T3Count;
    tc.configuration_id=config.configuration_id; tc.qualification_id=config.qualification_id;
    tc.usage=t::BatchUsage::CoupledForces;
    const auto tr=t3.InitializeJoined(tc,binding);
    if(tr.status!=t::BatchStatus::Success) return Failure(Status::ComponentFailure,tr.message);
    fe::NodalAssemblyView assembly;
    const auto begin=owner.BeginTrial(&token,&assembly);
    if(begin.status!=fe::NodalStatus::Ok) return Failure(Status::ComponentFailure,begin.message);
    const auto qb=qeph.AssembleAccepted(assembly);
    if(qb.status!=q::BatchStatus::Success) { Discard(); return Failure(Status::ComponentFailure,qb.message); }
    const auto tb=t3.AssembleAccepted(assembly);
    if(tb.status!=t::BatchStatus::Success) { Discard(); return Failure(Status::ComponentFailure,tb.message); }
    Discard();
    const auto pr=publication.Initialize(owner,qeph,t3);
    if(pr.status!=fe::ShellPublicationStatus::Success) return Failure(Status::ComponentFailure,pr.message);
    const auto read=owner.CopyAccepted({accepted.position.data(),accepted.velocity.data(),NodeCount,
        accepted.orientation.data(),accepted.omega.data()},&accepted.stamp);
    if(read.status!=fe::NodalStatus::Ok) return Failure(Status::ComponentFailure,read.message);
    const auto dr=publication.CopyAcceptedDiagnostics(accepted.stamp,&accepted.diagnostics.shells);
    if(dr.status!=fe::ShellPublicationStatus::Success) return Failure(Status::ComponentFailure,dr.message);
    initialized=true;
    return Success();
}
} // namespace crash::cases::source_part_elastic
