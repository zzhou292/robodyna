# Original robot demos through Robodyna

`//examples/robotics:native_demos` builds 11 original programs directly from their
retained `main` functions. The programs cover Curiosity and Viper rigid/SCM
terrain, Turtlebot, Little Hexy, industrial robotics, RoboSimian rigid/SCM terrain,
and the rigid-ground and SPH-soil Lander cases. The two Lander mains share one
original `model/Lander.cpp` compilation owner.

The complete 17-program roster remains in the initial `DemoCatalog.json`.
All programs now have actual targets and passed the current full compile matrix:
11 base cases, two Sensor cases, one Multicore granular case and three Viper CRM
cases. Use the matrix's receipts for current build status; the catalog retains
its initial declaration-time status fields.
Declaration and successful linking are separate from runtime qualification. The
interactive loops, timestep selection, contact equations, solver setup, graphics,
and original output behavior have not been changed. `lander_crm` reuses the SPH
solver and its CPU mechanics coupling; the other ten admitted demos use CPU
mechanics. No claim of CUDA rigid-body execution is made.

The baseline Vehicle profile keeps `CHRONO_CRM` off. The guarded declarations in
`chrono_vehicle/wheeled_vehicle/test_rig/ChWheelTestRig.h` at lines 110, 269, 386,
and 430 change its public methods and object layout; matching implementation
branches exist in `ChWheelTestRig.cpp`. `viper_wheel.h` also gates `CreateBCE`
behind that same profile. `demo_ROBOT_Viper_WheelSlope_CRM.cpp:205` calls it, so
even that direct-SPH demo requires the coherent Vehicle CRM profile.

The named `fsi-sph` profile configures transitive Vehicle headers, SCM/Vehicle
implementation owners and helper consumers together. Defining the
macro only for a demo would mix incompatible class layouts. Different native
profiles must not share a binding backend until that composition is qualified.
Binding compositions have separate runtime and ownership gates.

The compile batch does not close or stage all runtime assets. Read
[RUNTIME_ASSETS.md](RUNTIME_ASSETS.md) before running programs; inherited relative
paths and output directories still apply.
