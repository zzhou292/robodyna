# FMI examples

`native_demos` builds five real retained driver programs. `fmus` builds their
eight FMU models, including native shared libraries, generated descriptors and
ZIP packages. The extra template driver is included explicitly.

| Driver | Generated models |
| --- | --- |
| `hydraulic_crane_fmi2_cosim` | FMI 2 crane and actuator |
| `hydraulic_crane_fmi3_cosim` | FMI 3 crane and actuator |
| `van_der_pol_modex` | FMI 2 and FMI 3 Van der Pol |
| `hydraulic_crane_modex` | FMI 2 model-exchange actuator |
| `template_cosim` | FMI 2 template component |

See the [module contract](../../../src/integrations/fmi/README.md) for build,
resources, qualification gates and runtime limits. Use a fresh output directory
and the existing guard for execution; some original drivers are interactive.
No simulation runs merely by selecting the driver build aggregate, although the
FMU packaging action instantiates models to produce their real descriptors.
