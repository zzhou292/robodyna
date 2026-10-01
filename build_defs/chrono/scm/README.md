# Retained SCM terrain demonstration

The native terrain target compiles the imported `SCMTerrain.cpp`, `ChTerrain.cpp`,
`ChWorldFrame.cpp` and `ChVehicleDataPath.cpp` unchanged. The separate presentation
target adds the imported `ChScmVisualizationVSG.cpp`. Image decoding is provided by
one shared native STB compile owner, so a headless terrain program does not need
VSG and the graphics executable does not define the STB functions twice.

The selected example is the original
`src/demos/vehicle/terrain/demo_VEH_SCMTerrain_RigidTire.cpp`: its lugged tractor
wheel, 500 kg mass, inertia (20,20,20), angle ramp pi/4 rad/s, 0.04 m grid, 6 by 2 m
soil patch, location-dependent soil parameters, bulldozing and active domain are
retained. Its original physical step is 0.002 seconds. Bounded presentation hooks
must not replace any of those mechanics or change the normal interactive path.

This retention profile uses **CPU SCM with Bullet ray casting** and GPU Vulkan
rendering. SCM's optional HIP ray-casting backend is a different formulation with
a documented sinkage offset; enabling it would require a separate qualification.
The selected soil callback also makes its GPU contact-force path ineligible.

Planned run: 6 physical seconds (3,000 steps), 25 captured states per second plus
the initial state, yielding 151 frames. Record finite wheel state/contact forces,
travel, touched soil-node count and maximum soil depression/plastic sinkage.
Check the headless physics result before the guarded video run. Neither source
presence nor the height-query unit test proves the dynamic demonstration passed.

Native targets:

```text
//build_defs/chrono/scm:terrain
//build_defs/chrono/scm:visualization
//build_defs/chrono/scm:terrain_values_test
```

No old `Chrono_core`, `Chrono_vehicle` or `Chrono_vehicle_vsg` shared libraries
may satisfy the demonstration. The existing declared external VSG SDK and driver
remain the renderer's dependencies. All execution remains under the parent
workstation resource guard; these files do not start jobs themselves.
