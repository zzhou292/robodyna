# Retained original vehicle joint model

`VehicleJointModel` binds the authenticated TYPE45 source to the existing
`VehiclePhysicalModel` rigid/domain authority. It selects exactly the 38 required
rows, preserves their original order and indexes, and retains all six omitted
boundary declarations as source evidence. TL maps nodes/body groups and computes
only the actual initial mass/mean-principal-inertia damping inputs. Source
properties and geometry are reused without a second parser or constructor.

The model creates no owner, timestep, automatic stiffness or material history.
The later resident contributor must obtain authenticated post-transfer startup
coefficients from the actual owner and join its existing accepted transaction.
Original door/hood hinge release DOFs are retained in the TYPE45 property kinds.

Preflight rejects mismatched physical domain backing and inconsistent source
counts before packing/allocation. Its conservative complete budget sums upstream
physical/source phase reservations, the explicitly capped native model startup,
and local packing. Shared immutable backing is deliberately overcharged here;
these numbers are reservations, not a claim of actual RSS or new allocations.

The owning original gate checks every retained joint/endpoint against original
rows and actual plain/PART groups (48/28 endpoint occurrences), boundary exclusion,
exact native cap/retry and copy lifetime. TL's owning model gate separately
covers invalid geometry/properties/source identity and late failure.
Configure this directory with the original physical-model fixture paths,
Chrono_DIR and explicit TL root. Build `robo_dyna_vehicle_joint_model_check`;
CTest `vehicle_joint_model_original`. No CUDA or connected-crash qualification
is claimed by this immutable factory.
