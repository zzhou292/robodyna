# Native model parsers

`adams`, `opensim`, `urdf`, `urdf_vsg`, and `python` compile the retained adapters.
The YAML owners are `yaml_mbs`, `yaml_vehicle`, `yaml_cfd`, `yaml_sph`, `yaml_tdpf`
and the single selected-backend `yaml_fsi` factory. All use the existing native
mechanical system; these packages add no simulation state or clock.

The source inventory authenticates 28 unchanged files and thirteen unique parser
translation units. Optional source-built URDF and CPython embedding dependencies
are declared in [PARSER_SDKS.md](../../../build_defs/sdk/PARSER_SDKS.md).

Use separate coherent profiles for ordinary and YAML programs:

```text
bazel build //examples/io/parsers:native_demos
bazel test //src/io/parsers:source_inventory_test //tests/parsers:urdf_test //tests/parsers:python_test
bazel test --config=yaml //tests/parsers:yaml_test
bazel build --config=yaml --config=fsi-sph --config=fsi_tdpf //examples/io/yaml:native_demos
```

The YAML profile sets both the parser and VSG configuration consistently across
libraries and consumers. Several CFD parser headers contain VSG-dependent virtual
methods or members, so demo-only defines are insufficient. Default configuration
remains unchanged. The binding package currently admits only its qualified
baseline profile.

## Retained scenario admission

Source audit after all four URDF SDK source builds completed. Compilation and the
bounded integration tests remain separate from the interactive demonstration runs.

| Original YAML demo | Default scenario | Native mechanical solver |
| --- | --- | --- |
| MBS | `yaml/mbs/mbs.yaml` | Barzilai–Borwein |
| MBS controller | `yaml/mbs/mbs_controller.yaml` | Barzilai–Borwein |
| FEA | `yaml/fea/mbs.yaml` | MINRES |
| Vehicle | `yaml/vehicle/vehicle.yaml` | Barzilai–Borwein |
| FSI | Cylinder drop, SPH | Barzilai–Borwein |

All five defaults use retained built-in solvers. None requires PardisoMKL or MUMPS.
The optional TDPF sphere-decay input uses built-in GMRES. Explicitly selecting
PardisoMKL/MUMPS in another YAML file still requires a separately admitted coherent
solver profile; the original parser throws when the selected solver is unavailable.
No solver substitution is introduced by this build port.

Additional inherited behavior:

- FEA's default simulation requests HDF5 output. Initial core HDF5 output remains
  disabled, so the retained output settings code emits a warning and disables
  output. This does not disable dynamics or visual rendering. Actual TDPF HDF5
  model input remains owned by the TDPF dependency.
- `demo_YAML_fsi.cpp` prints seven choices but clamps nonempty input to `[1,5]`.
  Its TDPF case6 and arbitrary-file case7 therefore cannot be selected through the
  existing prompt. Both parser implementations can compile, but this does not prove
  that those two GUI choices work. The original control flow is unchanged; a narrow
  reviewed demo-only correction would be a separate task.
- GUI run lengths, model data paths and output directories remain inherited.
  Build success is not a bounded-run or physical-correctness receipt.
- The six ordinary parser demos use the baseline binary profile. The SolidWorks
  driver selects the actual owned Python package and shares the same native core
  DSO with its host. It cannot be mixed with an unqualified YAML/FSI binding layout.
