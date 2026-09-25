# Robo-dyna: Yaris delivery roadmap

Updated 2026-09-25. Autonomous implementation has resumed. The delivery is a Yaris crash into a finite mesh wall with self-contact, plastic deformation and reviewed videos, using TL-FEA mechanics and Chrono infrastructure. The user requires a GPU implementation faster than CPU OpenRadioss for matched physics and complete execution.

The workspace plan is `/home/jsonzhou/Desktop/chrono-work/planning/OPENRADIOSS_GPU_IMPLEMENTATION_PLAN.md`; current evidence and active branches live in `planning/IMPLEMENTATION_PROGRESS.md` and `HANDOVER.md`. The [architecture guide](ARCHITECTURE.md) owns stable module boundaries. Earlier status is preserved in [history](history/YARIS_DELIVERY_PLAN_2026-09-18.md).

## Qualified capabilities and scope

The existing selected assembly has CUDA explicit structural dynamics, native-tested shell plasticity, five solid families, beams, joints, connections, rigid groups and CIN. One TL physical owner advances accepted/trial state. Chrono replays accepted geometry with part colors.

The former frictionless fixed-triangle self-contact profile passed two strict 225 ns intervals and exact archive/replay qualification. It takes 200–255 seconds per interval. It differs from native TYPE25; it remains an experimental validation profile. Its 6,000-step long run was never launched and is not the selected production approach.

The new foundation includes a native-tested C++/CUDA TYPE25 normal response, ordered parallel solid assembly and an offline resolved-case/performance consistency gate. Coupled friction/history is under qualification. Native current geometry, coefficients, candidate retention and source history ownership remain required before selecting the new contact profile in the vehicle runtime. No production target may link OpenRadioss or invoke its Engine to advance mechanics. External reference executions and native donor wrappers are qualification tools only.

The measured solid-assembly improvement is 1.26×–3.11× against the previous GPU implementation on controlled workloads. It is not an OpenRadioss CPU or whole-vehicle speedup. The case parity gate currently rejects the actual full-native versus selected-V5 comparison because their physics and mass ledgers differ.

## Remaining delivery gates

| Gate | Required result |
| --- | --- |
| Native contact | Current Q4/T3 geometry, initial offsets, side stiffness/gaps, exclusions, retained search, secondary-row history and selected friction agree with the source oracle. Raw native flags are resolved explicitly; the Yaris printed edge1000 selects raw edge0. |
| GPU composition | Complete candidate coverage, history updates and force/STI gathering stay on device where useful and publish through the existing owner. Discard/retry preserves all accepted state. |
| Native step and model | Adaptive phase/limit semantics, mass/inertia/constraint ledgers and missing solid/discrete/auxiliary participants are reconciled with a declared native case. No hidden scaling or softening. |
| Matched performance | At least three interleaved repeats demonstrate faster complete GPU steps and complete-run time outside observed variation, with equivalent outputs and physical time. |
| Vehicle trajectory | Qualified wall+self impact extends through visible deformation, then approximately 5 ms, 20 ms and the 200 ms delivery horizon as runtime and physical evidence allow. |
| Reviewed videos | Closed accepted-state archives, Chrono overview/detail capture, stable part colors, physical scale1, actual timestamps, full decode and first/middle/final visual review. |

Complete energy/momentum/contact-work/load-path/failure ledgers and restart remain explicit capabilities to finish; visualization frames are not physical restart state. A prescribed-contact coupon or short prefix cannot establish full-vehicle completion.

## Preserved review material

The longest existing archive remains **wall-only**: `crash-work/runs/yaris-wallremoval-10000-1`, 10,000 steps / approximately 2 ms. It must not be changed or deleted. Reviewed overview and impact-detail movies live in `crash-work/renders/yaris-wallremoval-10000-review-1/`. There is no new full self-contact crash video yet.

Use existing resource guards and workspace/author locks. Keep other GPU work running, preserve paused branches and failed evidence, and do not push.
