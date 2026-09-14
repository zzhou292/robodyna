# Publication-validated self-contact activity

`SelfContactPhysicalActivity` retains one exact execution-backed physical
publication and maps the selected active-use parents from complete native
QEPH/T3/QBAT activity readbacks. Callers provide no activity bytes.

The host gate proves exact arena sizing, transition policy, public shape,
source wiring, and CMake/Bazel ownership. The opt-in CUDA gate must run only
under the owning workstation/GPU guard. It exercises the actual physical owner,
participant assembly, `PreparePhysical`, removal, long inactivity, forged
inputs, late readback rollback/retry, receipt lifetime, and allocation
stability.

Transaction integration is intentionally separate: accepted assembly consumes
an accepted activity receipt; candidate validation consumes a prepared activity
receipt. A raw `SelfContactActivityView` is a receipt-borrowed query result, not
authority.
