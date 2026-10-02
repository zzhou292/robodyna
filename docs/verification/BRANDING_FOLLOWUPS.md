# Branding provenance after later implementation work

The historical branding manifest and its qualification evidence remain pinned to
`e16909532138ea0cc2a57225a504cff05d0c5bbe`. They describe that checkpoint and are not
rewritten whenever a later source or build change is made.

`BRANDING_FOLLOWUPS.json` is the current, separate admission record for reviewed
changes to files also present in that historical manifest. The initial record
contains three paths: the owned compatibility-package BUILD metadata and two
retained Peridynamics example files with later include/declaration repairs.
The current record also includes the C# camera demo's two constructor-call
repairs. They explicitly supply the existing native denoiser/integrator defaults
while preserving the original flags, gamma and other camera settings. The
historical branding entry and complete import identity remain unchanged.

For imported sources, verification preserves the exact historical source-history
prefix, reverses only the later suffix to recover the historical branded bytes,
and still verifies the complete source inverse back to the immutable import.
It then applies the existing pre-branding and asset-retirement checks. The owned
BUILD file has a separate exact inverse to its historical branded snapshot;
it is not assigned a fabricated upstream import identity.

The checker does not modify files, refresh expected hashes or apply patches.
It rejects unlisted edits, repinned historical manifests, rewritten recipe
prefixes, missing inverse operations, stale entries, duplicate paths and symlink
inputs. Later changes need a reviewed update to this current registry. Preserve
the historical checkpoint hashes and append imported source-history operations;
do not repin original hashes to accept changed code.

The original three-argument Python verification API remains strict and can still
verify a historical checkout without follow-ups. The current CLI uses this registry
when present, or accepts an explicit `--followups` path. An explicitly requested
missing registry fails rather than being silently ignored.

The initial focused checks passed thirteen behavioral cases and the live branding
verification:472 presentation files,464 imported histories,25 retired assets,
three unchanged logo copies and three reviewed later edits. This is provenance
verification, not an optional-module runtime or physical-correctness claim.
The live checker also passed after the fourth, camera-call follow-up was added.
