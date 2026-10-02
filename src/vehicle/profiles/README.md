# Coherent optional Vehicle profiles

The existing Vehicle and SCM libraries remain the only owners of their original
translation units. Optional capability flags change their one shared generated
`ChConfigVehicle.h` and add exactly one original terrain implementation each:

| Configuration | Shared declarations | Additional owner |
| --- | --- | --- |
| baseline | CRM and OpenCRG disabled; CPU SCM retained | none |
| `--config=fsi-sph` | Core FSI/SPH enabled and Vehicle `CHRONO_CRM` enabled together | `//src/vehicle/crm:terrain` |
| `--config=opencrg` | Vehicle `CHRONO_OPENCRG` enabled | `//src/vehicle/opencrg:terrain` |

The original wheel-rig implementation is recompiled under the same configuration
as its consumers. It is not copied into a second library. This matters for the
CRM-guarded virtual methods, parameter fields and terrain creation/advance path.
`CRMTerrain::Advance` and its existing MBD callback retain their original order
and owner; no new clock or stepping loop is added by this build work.

OpenCRG uses the exact source revision named by the retained contributor recipe,
compiled natively through `@opencrg_sdk//:opencrg`. Its 11 C translation units and
Apache notices are retained. The CPU host test loads the original 22 m by 3 m
sample road and checks its height, normal, friction and System ownership without
stepping or opening graphics. The CRM host test covers the real profile-dependent
wheel-rig parameter constructor without allocating a fluid/device.

The source gate compares optional CMake sets with the baseline and authenticates
the retained terrain, rig and sample bytes. The existing aggregate ownership gate
has explicit optional flags and checks the added compile owners in the actual
dependency graph. Source declaration and host qualification do not replace the
separate guarded CRM GPU simulation gate.
