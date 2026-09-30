#include "SourcePartElasticInternal.h"
#include <algorithm>
#include <cmath>

namespace crash::cases::source_part_elastic {
Report SourcePartElasticCase::Impl::Initialize(const source::SourcePartContactFixture& input,const Config& settings) {
    source=input; config=settings;
    const bool plastic=config.material_model!=MaterialModel::ElasticLaw1;
    if(plastic&&(config.material.declaration().density_kg_m3!=source.surface_mass().density_kg_m3||
        config.material.declaration().thickness_m!=source.surface_mass().thickness_m))
        return Failure(Status::InvalidInput,"Source material density/thickness differs from the authenticated geometry");
    fe::ShellBatchPlasticityConfig plasticity;
    if(plastic) {
        plasticity.material_id=config.material.declaration().material_id;
        plasticity.curve_id=config.material.declaration().curve_id;
        plasticity.curve=config.material.curve();
        plasticity.rate=config.rate;
    }
    const auto cr=collection.Initialize(source);
    if(cr.status!=source::FixtureStatus::Ok) return Failure(Status::ComponentFailure,"Original source collection conversion failed");
    const auto br=binding.Initialize(collection.input());
    if(br.status!=fe::ShellBindingStatus::Success) return Failure(Status::ComponentFailure,br.message);
    if(binding.node_count()!=NodeCount||binding.qeph_count()!=source::Q4Count||binding.t3_count()!=source::T3Count)
        return Failure(Status::ComponentFailure,"Incomplete original source collection");
    accepted.position=source.coordinates();
    for(std::size_t n=0;n<NodeCount;++n) {
        const auto& m=binding.nodes()[n].native;
        inverse_mass[n]=1/m.mass; inverse_inertia[n]=1/m.isotropic_inertia;
        accepted.orientation[4*n]=1;
    }
    const auto loading=InitializeLoading();
    if(!loading) return loading;
    fe::NodalStateConfig state;
    state.node_count=NodeCount; state.fixed_dt=config.dt;
    state.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    std::array<std::uint8_t,NodeCount> free{};
    const auto nr=owner.Initialize(state,{accepted.position.data(),accepted.velocity.data(),accepted.omega.data(),
        NodeCount,accepted.orientation.data()},inverse_mass.data(),{free.data(),free.data(),inverse_inertia.data()});
    if(nr.status!=fe::NodalStatus::Ok) return Failure(Status::ComponentFailure,nr.message);
    if(config.experiment==Experiment::ElasticPulse&&
       (cudaMalloc(reinterpret_cast<void**>(&device_pulse),sizeof(pulse_force))!=cudaSuccess||
        cudaMemcpy(device_pulse,pulse_force.data(),sizeof(pulse_force),cudaMemcpyHostToDevice)!=cudaSuccess))
        return Failure(Status::DeviceFailure,"Pulse device setup failed");
    fe::ShellBatchStartup startup;
    if(config.experiment!=Experiment::ElasticPulse)
        startup={fe::ShellBatchStartupKind::ReferenceUniformTranslation,
            {config.initial_velocity[0],config.initial_velocity[1],config.initial_velocity[2]}};
    q::QephBatchConfig qc; qc.owner=owner.accepted(); qc.element_count=source::Q4Count;
    qc.configuration_id=config.configuration_id; qc.qualification_id=config.qualification_id;
    qc.usage=q::BatchUsage::CoupledForces;
    qc.startup=startup;
    const auto qr=plastic?qeph.InitializeJoined(qc,binding,plasticity):qeph.InitializeJoined(qc,binding);
    if(qr.status!=q::BatchStatus::Success) return Failure(Status::ComponentFailure,qr.message);
    t::T3BatchConfig tc; tc.owner=owner.accepted(); tc.element_count=source::T3Count;
    tc.configuration_id=config.configuration_id; tc.qualification_id=config.qualification_id;
    tc.usage=t::BatchUsage::CoupledForces;
    tc.startup=startup;
    const auto tr=plastic?t3.InitializeJoined(tc,binding,plasticity):t3.InitializeJoined(tc,binding);
    if(tr.status!=t::BatchStatus::Success) return Failure(Status::ComponentFailure,tr.message);
    fe::NodalAssemblyView assembly;
    const auto begin=owner.BeginTrial(&token,&assembly);
    if(begin.status!=fe::NodalStatus::Ok) return Failure(Status::ComponentFailure,begin.message);
    const auto qb=config.experiment==Experiment::ElasticPulse?qeph.AssembleAccepted(assembly):qeph.AssembleAccepted(owner,assembly);
    if(qb.status!=q::BatchStatus::Success) { Discard(); return Failure(Status::ComponentFailure,qb.message); }
    const auto tb=config.experiment==Experiment::ElasticPulse?t3.AssembleAccepted(assembly):t3.AssembleAccepted(owner,assembly);
    if(tb.status!=t::BatchStatus::Success) { Discard(); return Failure(Status::ComponentFailure,tb.message); }
    Discard();
    const auto pr=publication.Initialize(owner,qeph,t3);
    if(pr.status!=fe::ShellPublicationStatus::Success) return Failure(Status::ComponentFailure,pr.message);
    const auto read=owner.CopyAccepted({accepted.position.data(),accepted.velocity.data(),NodeCount,
        accepted.orientation.data(),accepted.omega.data()},&accepted.stamp);
    if(read.status!=fe::NodalStatus::Ok) return Failure(Status::ComponentFailure,read.message);
    const auto dr=publication.CopyAcceptedDiagnostics(accepted.stamp,&accepted.diagnostics.shells);
    if(dr.status!=fe::ShellPublicationStatus::Success) return Failure(Status::ComponentFailure,dr.message);
    if(config.experiment!=Experiment::ElasticPulse) {
        const auto& k=accepted.diagnostics.shells.kinetic;
        initial_kinetic=k.translation+k.rotation;
        if(!std::isfinite(initial_kinetic)||initial_kinetic<=0)
            return Failure(Status::ComponentFailure,"Moving source startup lacks measured positive common kinetic energy");
        accepted.synchronized_velocity=accepted.velocity;
        accepted.synchronized_omega=accepted.omega;
        accepted.diagnostics.synchronized_kinetic=initial_kinetic;
    }
    accepted.plastic.enabled=plastic;
    if(plastic) {
        const auto observed=InitializePlasticObservation();
        if(!observed) return observed;
    }
    initialized=true;
    return Success();
}
} // namespace crash::cases::source_part_elastic
