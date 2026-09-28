// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "FullLedgerRig.h"
#include "lib_src/collision/radioss_type25/activity_source/Types.h"
namespace type25_source_test {
struct ShellRemovalRig : FullLedgerRig {
  explicit ShellRemovalRig(double failure_strain=1e-3):FullLedgerRig(true,failure_strain){}
  n::activity_source::Controls deletion{n::activity_source::Deletion::ContainingElement,false,s::SolidErosion::Disabled};
  void InitializeRemoval() {
    Initialize(false);
    auto config=Config();config.activity=n::ContactActivityPolicy::ShellRemoval;
    auto source=Source();source.activity_controls=&deletion;
    Check(contact.Initialize(config,source,owner,publication,fixture.physical,Participants(),Identity()));
    Check(publication.ConfigurePhysicalScratchParticipation(owner,fixture.physical,Participants(),Identity(),{{},contact.roster_entry()}));
  }
  void Warm(unsigned count=2) {
    for(unsigned step=0;step<count;++step) {
      FullLedgerAttempt a;Begin(a);Check(contact.AssembleAccepted(owner,a.token,a.assembly));
      Prepare(a);Seal(a);Check(Commit(a));
    }
  }
  void RemovalLoad(FullLedgerAttempt& a) {
    const auto node=fixture.domain.Find(14);
    const double force=2*m[node]*.001/(Dt*Dt);
    Check(cudaMemcpyAsync(a.assembly.forces.force_x+node,&force,sizeof(force),cudaMemcpyHostToDevice,a.assembly.stream));
    Check(cudaStreamSynchronize(a.assembly.stream));
  }
};
}
