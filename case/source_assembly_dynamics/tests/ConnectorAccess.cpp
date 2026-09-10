#include "Fixture.h"
#include <limits>

namespace crash::cases::source_assembly_dynamics {
Report SourceAssemblyDynamicsTestAccess::RejectConnector(SourceAssemblyWallCase& run,ConnectorFault fault) {
    auto& s=*run.impl_;auto r=s.Prepare();if(!r)return s.Stop(r);
    r=s.Evaluate();if(!r)return s.Stop(r);
    if(!s.connector)return s.Stop(Failure(Status::ComponentFailure,"Missing test connector"));
    auto& last=s.connector->results[1-s.accepted_slot].back();
    if(fault==ConnectorFault::Work)last.history.internal_work_J[3]+=.01;
    if(fault==ConnectorFault::Phase)++s.candidate().diagnostics.shells.connector.source_instance_id;
    if(fault==ConnectorFault::LastForce)last.endpoints[1].couple_Nm.z=std::numeric_limits<double>::infinity();
    r=s.Check();
    if(r)return s.Stop(Failure(Status::ComponentFailure,"Injected connector candidate unexpectedly passed"));
    return s.Stop(r);
}
} // namespace crash::cases::source_assembly_dynamics
