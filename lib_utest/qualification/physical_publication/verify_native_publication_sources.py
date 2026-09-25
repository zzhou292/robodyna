#!/usr/bin/env python3
"""Closed native selector publication, not numerical-contact qualification."""
from pathlib import Path
import re
root=Path(__file__).resolve().parents[3]
def source(name):
    text=(root/name).read_text()
    return re.sub(r'/\*.*?\*/|//[^\n]*','',text,flags=re.S)
state=source('lib_src/elements/publication/NativeContactPublicationState.cpp')
header=source('lib_src/elements/publication/NativeContactPublicationState.h')
transaction=source('lib_src/elements/publication/PhysicalTransaction.cpp')
issuer=source('lib_src/elements/publication/ShellPhysicalScratchParticipation.cpp')
for text in (state,header):
    for forbidden in ('std::function','cudaMalloc','cudaMemcpy','cudaStreamSynchronize','std::vector'):
        assert forbidden not in text, forbidden
assert header.index('private:')<header.index('bool Attach(')<header.index('bool Stage(')
assert 'NativeContactPublicationState(NativeContactPublicationState&&)=delete' in header
assert 'trial_identity::SameStamp(stamp,accepted_stamp_)' in state
assert 'trial_identity::SamePrepared(prepared_,view)' in state
body=state.split('void NativeContactPublicationState::Publish(',1)[1].split('{',1)[1].split('}',1)[0]
assert re.sub(r'\s+','',body)=='accepted_=staged_;force_base_stamp_=accepted_stamp_;force_phase_available_=true;accepted_stamp_=stamp;++generation_;Discard();'
commit=transaction.index('nodal = owner.Commit(token)')
publish=transaction.index('PublishNativeContactState(stamp)')
assert commit<publish and 'if (nodal.status != NodalStatus::Ok) return fail(Nodal(nodal));' in transaction[commit:publish]
consume=issuer.split('void ShellPhysicalScratchParticipation::Consume()',1)[1].split('{',1)[1].split('}',1)[0]
assert 'DiscardTrial' not in consume
assert 'native_contact_->Ready(owner,authentic,issuer.generation_)' in issuer
assert 'native_contact_->Discard()' in issuer
print('Closed native contact publication source proof: PASS')
