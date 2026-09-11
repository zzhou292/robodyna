# Pinned caller and explicit case mapping

All native source below is OpenRadioss commit
`a62b27e6baa555d222a580d6218867d0be4d70b5`. Complete qualification donors and their
Git-blob/SHA256 identities are in `native/source-manifest.json`. Additional reader
evidence is retained in workspace `crash-work/deps/openradioss-rubber-startup-1/`
(`wedge-source-manifest-{1,2,3}.json`) and existing authenticated source caches.

* The literal original solid card CFG reads eight slots without deduplication:
  `hm_cfg_files/config/CFG/Keyword971/ELEMENTS/solid.cfg:140-146,178-184`.
  `sdiModelViewPO.h:510-578` counts positive occurrences and returns them in order;
  `CreateElement:1648-1718` stores the passed IDs. The selected LSDyna model does
  not override these element methods.
* `convertelements.cxx:78-81,110-145,282` preserves all eight IDs for this pattern.
  `cpp_brick_read.cpp` copies the received eight IDs into IXS. In
  `hm_read_solid.F:145-219`, the six-node branch requires zero last two slots;
  repeated positive slots remain ISOLNOD8. Thus a claim that these literal cards
  automatically dispatch to S6Z, or automatically trigger its six-node guard,
  is unsupported by this trace. No complete original-import execution is claimed.
* For an explicitly supplied compact six-node packet, `hm_read_solid.F:170-193`
  embeds it and corrects negative orientation. `INITIA:1087-1118` requires JHBE24
  for the six-node TYPE14/6 branch before calling S6ZINIT3. An already compact
  six-node packet with JHBE1 is incompatible with that guard.
* Original HGID2000017 is IHQ2/QM0.1, and the converter's PART hourglass override
  selects Isolid1 despite the blank-ELFORM default24. The demo deliberately
  selects HEPH24 for bricks and the native S6Z profile for the explicitly mapped
  wedges. Original cards remain authoritative source evidence; they are not
  rewritten to imply native converter equivalence.
* `defaults_mod.F90:141` initializes solid IMAS to zero. `hm_read_defsolid.F`
  can override it; this selected profile fixes IMAS0 explicitly.
  `S6ZINIT3:232` reads that value and passes it to S6MASS3. `S6FRACA:58-89`
  computes angular weights only for positive IMAS; otherwise its weights are
  ONE. `S6MASS3:69-93` writes six mass contributions in native slots 1,2,3,5,6,7,
  each FILL*GBUF-rho*VOLU/6, and adds SIX*MASS to the part total. It adds no
  scalar nodal inertia. No physical inertia is derived by subtracting floors.
* The ISMSTR10 branch in `S6ZINIT3` calls S6ZCOOR3/S6ZJACIDP before its unconditional
  S6ZRCOOR3 call. The inverse reference Jacobian uses world double coordinates;
  the local volume, VZL and length then use the cyclic frame. The NIP1 centered
  initial volume is `.5*2*(VOL+VZL*0)`. This reference retains the consumed values;
  complete material/force/state and reference-vector ownership remain later work.
* `S6ZDERI3:175-179` deliberately calls SLEN's first face with X/Z order 1,2,5,4
  and Y order 1,2,4,5. The port retains this source expression and the separate
  two-small-faces length multiplier. The native test distinguishes the otherwise
  tempting all-coordinate 1,2,5,4 substitution.

Working-unit native and SI startup may differ by bounded conversion roundoff.
Source IDs/counts, shape selection and mass roles remain exact. No whole-model
native reduction order, raw-import compatibility, or completed crash is implied.
