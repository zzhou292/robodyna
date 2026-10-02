# Distributed agent synchronization

The existing `:synchronization` owner compiles34 original translation units,
including generated FlatBuffers/CDR message code. Vehicle, Robot and mechanics
implementations are reused through their current owners. Nine original demo mains
are declared in `//examples/distributed:native_demos`: six MPI and three DDS.
Source retention, compilation and actual distributed runtime results are separate.

The standalone FastDDS2.4.0, FastCDR1.0.24 and foonathan0.7.3 SDK matches the
retained build recipe. ROS uses a different FastDDS2.6 ABI in its separate node
process. This package must not load that ROS middleware into the distributed
simulation. The source/ownership gates check the implementation set and imports.

The message gate exercises real retained FlatBuffers round-tripping and the
original generated DDS serializer against the admitted FastCDR implementation,
including empty data and insufficient capacity. It does not create a DDS domain
or broadcast messages. The MPI gate runs the original `SynMPICommunicator` on
two local ranks for three message rounds, with unequal payload lengths, routing
and quit-state checks, reset and empty messages. The original communicator owns
MPI initialization and finalization. The shared `tools/mpi` harness provides the
bounded local launch and cleanup under the outer workstation guard.

Compilation/runtime qualification is pending the root guarded gates. The first
MPI coupon verifies message exchange, not distributed dynamics, scalability or
DDS discovery. Original interactive demos still require admitted data paths,
rank/process budgets and, where selected, a display. DDS demonstrations require
an isolated discovery/transport configuration before execution.

The retained MPI implementation calls `reserve` before writing gathered bytes
through `vector::data()` without updating vector size. The bounded coupon does
not establish general allocator/container correctness or sanitizer qualification
of that inherited behavior. No source algorithm is silently changed in this build
integration; any repair must have its own review and regression evidence.
