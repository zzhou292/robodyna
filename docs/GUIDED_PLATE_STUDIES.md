# Guided plate contact study

This is a six-node, two-Q4 elastic qualification fixture against the original
finite Yaris wall mesh. It is not a vehicle model. TL-FEA owns state, shell and
contact forces, admission and stepping. Chrono provides reference setup and
accepted mesh playback. No second clock or solver is introduced.

`GuidedPlateStudy` observes accepted scalar metrics at every step and captures
201 common-time geometry samples. Its three source modules separate physical
observables, transactional recording and refinement comparisons.
`GuidedPlateStudyIO` reads/writes bounded reports; `GuidedPlateArtifacts` owns
the optional renderable archive, with shared shell fields and canonical wall
utilities. `guided_plate_main.cpp` only composes these operations.

With guided plate checks enabled, the executable supports:

```text
robo-dyna-guided run WALL-MANIFEST NEW-STUDY-JSON 1 [NEW-ARCHIVE-DIRECTORY]
robo-dyna-guided run WALL-MANIFEST NEW-STUDY-JSON 2
robo-dyna-guided run WALL-MANIFEST NEW-STUDY-JSON 4
robo-dyna-guided compare COARSE-STUDY FINE-STUDY NEW-COMPARISON
```

Run these through the workspace resource guard and shared workstation lock.
All output paths must be new, with existing parent directories. The baseline
archive records every accepted interval and fixed-cadence frames at every 100
steps plus the final state. Refinement reports instead sample at exactly
`ceil(j*base_steps/200)*refinement` for `j=0..200`. Their schedules are explicit
and independent. An occasional coincident sample/frame causes two captures of
the same accepted state; neither advances mechanics.

Completed reports retain a missing-rebound outcome. Completion alone is not a
passing comparison: `compare` returns 2 and writes `passed=false` for a valid
experiment that fails a frozen response, energy, event or deformation gate.
Malformed inputs return 1. Force peak certificates enclose the maxima over all
accepted endpoints; they do not claim a continuous-time peak. Applied-base
forces determine impulse. Endpoint force fields describe the saved geometry.
Contact numerical estimates may lie outside their truth interval when the
declared error radius covers both endpoints.

The wall transform helper authenticates the original source, retains explicit
derived IDs and validates its complete finite mesh. Prescribed-state invariance
tests pass for flipped pairs and subdivision; independent variant trajectories
and the full impact/refinement/rendering gates remain pending.
