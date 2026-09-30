#pragma once
#include "AcceptedReplaySourceAssemblyConnectorValues.h"
namespace crash::output::connector_test {
inline void Put(Document& d,const char* key,Value value) {d.AddMember(Value(key,d.GetAllocator()),value,d.GetAllocator());}
// Authenticated source plus synthetic value records: exercises parser contracts,
// never a published solver state or an accepted seven-part trajectory.
struct Fixture {
    Fixture();
    replay_detail::Bundle bundle;
    Document configuration,frame;
    void Admit();
};
}
