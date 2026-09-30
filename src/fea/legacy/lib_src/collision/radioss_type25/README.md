# Selected OpenRadioss TYPE25 normal response (P1)

This is a pure normal-response packet, not a complete contact solver. Production
has no OpenRadioss library, executable, Fortran or GTest dependency. Search,
projection, native side stiffness/gaps, initial-offset lifecycle, tangential
friction, force assembly and global step control remain outside this module.
It must not replace the current vehicle contact before those gates are complete.

The numerical donor is OpenRadioss a62b27e6baa555d222a580d6218867d0be4d70b5,
engine/source/interfaces/int25/i25for3.F. Siemens copyright and AGPL-3.0-or-later
apply; full donor license is retained in LICENSE.md. Selected blocks preserve:

- history force-jump control (282–326), with distinct previous/staged slots;
- no-adhesion symmetric spring and elastic energy (339–378);
- IVIS2=1 harmonic-mass damping and stiffness contribution (413–452 and its
  zero-normal-damping/friction-viscosity continuation);
- signed normal damping-work history (691–709).

The actual Starter packet selects stiffness formulation4, initial penetration5,
damping flag1/factor0.05. Actual fresh no-target-scaling native Engine observation
at I25FOR3 entry proves KDTINT=IDTMINS=IDTMINS_INT=0 and double precision. Its
inspection receipt and source-derived packet are pinned under the owning
qualification/native/original. It does not prove the control flags of every case.
Other explicitly provided supported control values exercise the alternate source
arithmetic; the kernel itself does not add mass or select a global timestep.
Default EngineControls are unknown and reject, so no caller silently inherits a
vehicle-specific mode. Precision4, adhesion, prescribed-force and other stiffness
formulations are explicitly unsupported.

## Small API and state

Include collision/RadiossType25Normal.h. EvaluateNativeNormal consumes explicit
ResolvedNormalConfig, NativeNormalInput and NativeNormalHistory, and stages a
NativeNormalResult. EvaluateSiNormal adds an explicit UnitScale. Both use the
same host/device equations and publish only a complete finite successful result.
The caller owns input/output storage, pair identity, accepted/staged history and
commit/discard. An output's prior history can be used as input because all reads
precede publication. No allocation, synchronization, hidden global or source-ID
special case is used.

NativeNormalInput.dt is **native DT1**, the history/damping interval, not an
arbitrary proposed DT2. An adaptive owner must resolve that phase correctly.
A zero DT1 is admitted at this numerical substage because the donor explicitly
sets its reciprocal to zero; this does not admit a zero physical solver step.
The four interpolation weights and nodal masses are caller-prepared native values,
not new closest-point or effective-mass choices. Incoming stiffness is native
STIF before the force-jump control, symmetric HALF and damping augmentation.
The caller must supply penetration AFTER native initial-offset processing. This
module does not manufacture or update that offset.

History previous/staged stiffness is retained BEFORE the symmetric HALF factor.
force_stiffness records the elastic-force stiffness; stability_stiffness records
the final native STIF. The separate KT/CF channels exist only when terms_valid.
Invalid channels are defined as zero in this API and excluded from any claim
about unspecified donor scratch. Native signed damping can reverse FNI; there
is deliberately no generic unilateral clamp. damping_work is this substage's
increment, not a complete contact-energy ledger or universal positivity claim.
At zero penetration, the native substage clears weights, leaves all history
unchanged and leaves incoming STIF unchanged. The full pair lifecycle may still
have work to do elsewhere.

## Units and numerical domain

All native equations retain original working-unit floors. NativeConstants follows
constant_mod.F/MYREAL8: EPP=ONE/EP10 and EM30=ONE/(EP20*EP10). EM30 is used in
both stiffness and mass denominators; converting it once as a generic SI epsilon
would be wrong. The SI adapter converts each dimension to native units, evaluates
there, and converts each output back separately.

The observed wrapper uses mm,seconds,metric tonnes: UnitScale{0.001,1000,1}.
Therefore forceunit=1N, stiffnessunit=1000N/m, energyunit=0.001J,
dampingunit=1000kg/s, and native EPP=1e-10mm=1e-13m. This scale is an explicit
caller value, not a production default. Native and SI packet types differ at
compile time. Tests include a branch that fails if EPP is interpreted as metres.

The admitted packet has finite nonnegative penetration/stiffness/masses/time/DT1,
finite weights/normal velocity/history, and nonnegative damping parameters.
Nonfinite arithmetic or a negative resulting effective mass rejects the result;
these checked-domain rejections are not claims that the unrestricted Fortran
routine reports the same error. Physical callers must also establish geometry,
weights/history ownership and supported source flags. Compilation requires
binary64 round-to-nearest, no fast-math/FMA reassociation or flush-to-zero.

The independent oracle
compiles exact pinned Fortran blocks with only a scalar input/output wrapper and
observation assignments; production is not invoked by that oracle. See the
owning qualification README for exact boundaries and commands.
