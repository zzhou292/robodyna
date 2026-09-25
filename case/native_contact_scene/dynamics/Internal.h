#pragma once
#include "NativeSceneDynamics.h"
#include "case/vehicle_runtime/Packing.h"
#include "case/vehicle_runtime/Reports.h"
#include "lib_src/solvers/NodalCinRuntime.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace crash::cases::native_scene::dynamics_detail {
namespace f=tl::fea;
using vehicle_runtime::detail::RequireSuccess;
f::NodalStateConfig OwnerConfig(const PhysicalSource&,const DynamicsConfig&);
f::NodalStamp DescriptiveStamp(const PhysicalSource&,const DynamicsConfig&);
f::NodalCinStartup Cin(const PhysicalSource&,const DynamicsConfig&,const double* mass=nullptr,const double* inertia=nullptr);
f::NodalCinWitnessSource Witnesses(const PhysicalSource&);
f::qeph::QephBatchConfig QuadConfig(const PhysicalSource&,const DynamicsConfig&,const f::NodalStamp&);
f::t3::T3BatchConfig TriangleConfig(const PhysicalSource&,const DynamicsConfig&,const f::NodalStamp&);
f::ShellPublicationLimits PublicationLimits(std::size_t nodes);
}
namespace crash::cases::native_scene {
struct NativeSceneDynamics::Storage {
    Storage(const ContactSource& s,DynamicsConfig c,DynamicsForecast f):source(s),config(c),forecast(f){}
    ~Storage(){Discard();}
    ContactSource source;DynamicsConfig config;DynamicsForecast forecast;
    tl::fea::FENodalState owner;
    tl::fea::qeph::QephBatch qeph;tl::fea::t3::T3Batch t3;
    tl::fea::ShellBatchPublication publication;
    native::Transaction contact; // Reverse destruction: contact, publisher, batches, owner.
    tl::fea::NodalTrialToken token;
    tl::fea::NodalPreparedView prepared;
    tl::fea::NodalAssemblyView assembly;tl::fea::NodalCinAssemblyView cin;unsigned stage=0;
    NativeStepObservation observations[2];unsigned selected=0;bool pending=false,trial=false;
    tl::fea::ShellPhysicalParticipants Participants(){return {&qeph,&t3};}
    tl::fea::ShellPhysicalPublicationIdentity Identity() const{return {config.configuration,config.qualification,source.physical_source().startup()};}
    void Initialize();
    void BeginMaterials();
    void AssembleContact();
    void FinishPrepare();
    void Discard() noexcept;
};
}
