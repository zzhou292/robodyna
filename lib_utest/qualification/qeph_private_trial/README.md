# Private QEPH trial storage

The unchanged 10a/69 full vehicle binary showed a 520 MiB current-device global
memory increase between the QEPH CandidateElements launch and return. Its
compiled frame was 7184 bytes/thread with 255 registers. These are observed
resource facts, not proof of exclusive allocation ownership or solver timing.

This slice sends the existing force body to the already allocated, unpublished
trial ForceTrial. It borrows accepted shell/section values, removes nested full
history/trial copies, and supplies one section result workspace. Every retry
resets all force fields. Public value and existing host dispatcher entrypoints
still stage locally and publish only success, including output/history aliasing.
Accepted and trial device slabs remain disjoint under the existing owner gates.
No device allocation, launch size, material/failure law, owner, clock, metric or
numeric compiler flag changes. GeometryWork and its pre-CNDT3 kinematics remain
unchanged. The rigid-skin branch is unchanged pending measured resources.

Adapter::Publish only copies independent section outputs. Moving that copy to
after successful force computation cannot change the force inputs. Sidecars use
the existing ProposedPlasticSection/PublishFailureSection helpers; failure may
leave private force scratch incomplete, but batch status/finalization rejects
publication. The original host dispatchers retain their stronger output contract.

Four focused host groups compare all scalar force, history, kinematic, metric and
diagnostic fields, all LAW/failure/placement/OFF sidecars, poison/retry, late
coefficient failure and public accepted-history aliasing. One added real QEPH
owner rejection/retry group checks the entire common owner/material/frame state.
Existing resident TAB1 native recurrence, mixed Q/T CUDA owner, original layered
LAW44 recurrence, and global/layered LAW1 native/CUDA groups remain owning controls.

verify_sources.py reconstructs each of three pinned69 force bodies by reversing
only named staging/reference edits with exact unique anchors. Original validation
and arithmetic must match byte-for-byte. Public/private dispatch bodies are pinned
separately after review. This is source evidence, not a numerical test substitute.
The inherited diagnostics manifests also receive the exact already-qualified
projection-working-length identity comparison, with prior hashes retained.

Qualification must first inspect actual CandidateElements STACK/REG with the
same Release/no-FMA options; aggregate initialization is not assumed optimized.
Then run the named host/native/CUDA regressions. Only a later same guarded full
vehicle run can establish reduced actual device growth. No concurrency, register,
stack-limit, capacity or guard workaround is part of this change.
