# Version3 rigid/CIN contact source and observation boundary

2026-09-25. Isolated app source from qualified0f7. Accepted-owner TYPE25 mass
policy b192 is separately qualified (11host/21CUDA/2Bazel); it does not admit this
new source or a native coupled trajectory. No new Starter/Engine run has occurred.

## First declaration: rigid patch

`benchmarks/native_contact_scene/rigid_patch_capped.json` is version3 with explicit
all-shells contact and rigid-patch coupling. Physical coordinates, source IDs,
material, thickness, velocity and300ns initial/max cap are copied unchanged from
the qualified version2 declaration:18physical nodes,8wall T3 and4patch Q4. No
parameter is tuned to a reference trajectory. Wall nodes remain fully fixed;
all9patch nodes belong to one rigid part. The existing tilt and horizontal speed
provide an asymmetric impact without invented loads.

The reference deck declares a converter-style primary separately from physical
mesh metadata. It is node19, derived as max physical NID+1 in this closed source.
Its location is the mean of unique patch nodes in native converter first
encounter element/corner order (10,11,14,13,12,15,17,16,18). It is not a mass-weighted
centroid or an assumption about the later rigid center. Exact pinned
`ConvertUtils::GetCentroid(PartRead)` is the provenance for this order.

The group uses bodyID1/physicalPart2, Ispher2, ICoG1, skew/sensor0, no added
cross-inertia, no failure and no AMS extension. Mass/Jxx/Jyy/Jzz=1e-20 are literal
native converter input placeholders from `convertrigids.cxx196–225`. They are
labeled as such in `reference_rigid_body`; they are never inserted into TL's
physical nodal ledger, contact source, structural connectivity or archive mesh.
The source explicitly initializes both physical members and the auxiliary
primary with the same declared patch translation (separate group3). This velocity
choice is an explicit scene control, not an inferred converter default.

The native primary/group remains an authentic reference node and will be captured
separately. Its input placeholders must not be assumed to survive INIRBY as
independent physical mass. Full INIRBY primary/member M/J, center/frame and member
OFF observations decide the correspondence to the existing typed PART model.
HM_READ_RBODY's covered-shell negative-OFF path is source-proven, but its actual
resolved instance remains an observation gate. The TL factory must declare
`RigidSkin` from genuine coverage and use the existing PART/ledger model; it must
not relabel an ordinary constitutive material after runtime initialization.

Version1/2 bytes are preserved: the new nullable coupling field is omitted from
legacy exports, and the old native writer paths are unchanged. Exact version1
golden tests remain; a version2 baseline hash set was captured before these edits.
The existing C++ DeclaredSource currently rejects version3. That is intentional:
this slice is source/export only, not an accidental shipping physics admission.

## Native control and startup probe specification

Before any invocation, freeze the declaration/export/observer source and exact
Starter/Engine binaries and create a fresh output directory. Use existing owned
child/ptrace observation and workstation guards:4CPUs/10GiB RSS/32GiB reserve,
120s,64MiB output cap,NTHREAD1. Root reviews the concrete recipe and hands off the
lane before execution. No captured data becomes a shipping source table.

The first probe must establish:

- Physical source NIDs1..18 versus auxiliary primary19 using actual ITAB; full
  native node count and rigid source ID/group/member order, no truncation of the
  primary or accidental insertion into TYPE25 NSV/main connectivity.
- Source-derived /RBODY resolved Ispher/ICoG/skew/mass controls; raw M/J before
  rigid aggregation and after INIRBY for all members and primary, complete center,
  principal inertia/frame and initial V/VR. Retain input versus resolved values.
- Physical shell group OFF and source parent correspondence; all physical M/J
  and startup contact K/gaps still come from their actual source stages.
- TYPE25 ILEV1, complete primary/expanded maps, NSV, source removals, actual
  Irem_i2=1 and existing damping/friction/ASS0 controls. Expected intended source
  extent is12primary/24expanded faces and18physical secondaries; observation,
  not this expectation, decides admission.
- First MAINF/FOR3 phase X/V/VR, MS/MSI, ordered main MS/H and contact histories;
  primary/group force reduction and recovered member state at the subsequent
  accepted endpoint. Read response scratch only for source-defined active lanes.
- Actual DT1/DT12 grid under declared DTIX300ns, including epoch0 and half-kick.
  Cap choice is a declared coupon policy, not target mass scaling or the future
  matched four-thread vehicle benchmark.

Only after those controls/source phases close should a fresh full1000-interval
sequence be requested. Compare force/history n to native return n and physical
state/rawM n+1 to next native entry; capture VR/AR rather than claiming six-DOF
validation from the older X/V-only fixture. Use existing common publisher,
accepted activity source bridge, owner mass view and accepted archive/replay.

## CIN followup declarations

Next add a separately declared free master shell patch and genuine slave shell
nodes, with positive source element M/J and actual ILEV28 attachment controls.
No fixed master, rigid/CIN intersection, fake witness, added nodal mass or hidden
removal exception is permitted. Use existing classified attachment/source and
accepted-parent activity upload. The V5-compatible variant keeps Irem_i2=1;
positive wall contacts exercise changed master masses while tied secondaries may
be removed by native source filtering.

A different named diagnostic can explicitly choose Irem_i2=3. The pinned TYPE25
CFG states that3 leaves tied secondaries unchanged, while1 removes them; the
reader stores temporaryIPARI83 only for1. Positive zero-MSI contact still requires
actual resolved roster/filter and FOR3 observation. It is not V5 admission. These
CIN variants are planned, not accepted by the current rigid-only version3 parser.
