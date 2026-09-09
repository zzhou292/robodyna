# Source-scale shell formulation screen

The optional host screen compares the existing coherent Chrono/TL Reissner
stiffness with two explicit inertia choices. It helps decide whether that
element can advance toward the original Yaris sheet-metal parts. It does not
advance dynamics, flatten source elements, interpret MAT024, or admit a vehicle.

Enable `ROBO_DYNA_ENABLE_THIN_SHELL_SCREEN=ON` with the existing
`ROBO_DYNA_ENABLE_ELASTIC_COUPON=ON` reference foundations and FEA-enabled coherent
Chrono core. Build the selected targets with one worker under the workstation
guard. The screen itself runs on the host with one math-library worker.

```text
robo-dyna-thin-shell-screen PROVENANCE-JSON NEW-REPORT-JSON
```

The provenance input is a bounded JSON source/build inventory. The report embeds
both its exact-byte SHA-256 and its readable contents. Retain the matching source,
binary, linked Chrono library and guarded launch evidence with that inventory;
the executable does not claim a hermetic build. Output must be new. Exit 0 means
the frozen host screen passes, 2 means numerical rejection with all six diagnostic
records saved, and 1 means no completed report. Every result declares
`simulation_ready=false`.

The six fixtures have two Q4s, six physical nodes and 24 unconstrained coordinates.
Their element edges are 10 and 20 mm, each at half, equal and twice the source
1.648 mm thickness. E=200 GPa, density=7890 kg/m³, nu=0.3, shear factor=5/6 and
drilling stiffness factor=0.01 are explicit synthetic elastic overrides.

The original mass uses physical thickness inertia and the existing equal
numerical drilling policy. The counterfactual adds `m_ei*A_e/12` at each element
node using the whole element area, then assembles shared-node contributions.
This follows the centered QEPH inertia expression at the pinned OpenRadioss
reference; combining it with Reissner stiffness does not implement a QEPH element.
Physical inertia, original artificial drilling and added tangential/drilling
terms remain separate in the report.

`ElasticCouponModel` supplies actual reference setup and force evaluation.
`ShellPatchAudit` supplies reusable perturbations, derivatives and audits.
`ShellPatchInertia` assembles the explicit inertia alternatives.
`ShellModeComparison` uses common physical translational mass for mode identity.
`ThinShellScreen` coordinates the fixed fixtures; separate spectrum, modal-energy
and report modules keep numerical work and serialization independently editable.

Raw coarse/fine Chrono and fine TL force derivatives remain visible even if the
existing symmetry, parity, positivity, derivative uncertainty or eigen-residual
audit rejects. Raw symmetric eigenvectors are diagnostic data; the separate
`reference_passed` flag identifies an admitted reference spectrum.

The tracked branch is the lowest physical baseline frequency cluster. It must
pass the frozen bending-participation and coherent-tip checks. Mode identity
uses squared MAC or the minimum squared principal-angle cosine, at least 0.99;
geometrically ambiguous or rank-deficient matches fail. The tracked FD frequency
change must be at most 0.5%, and the matched inertia-counterfactual change at most
5%. No lower nonbending mode is skipped to obtain a favorable result.

Every mode retains physical/artificial kinetic partitions. Actual TL transverse
shear energy at a small declared amplitude and half that amplitude is a separate
finite-amplitude diagnostic. Neutral spectral, membrane-wave and physical rotary
step estimates remain separate. A favorable screen still needs covariance,
dynamics/refinement, warped Q4/T3, source material and attachment qualification.
