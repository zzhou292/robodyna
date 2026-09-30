# General initialization allocation phases

The source-produced seed requires only the primary transaction arena. It copies
history into both existing slabs, ICONT into both secondary slabs, and finalized
corner gaps into Main. The paired candidate inventories, paired maintenance
workspaces and incidence workspace neither produce nor consume that handoff.

After the existing main-arena source upload and stream drain, GeneralInitialize
will call the unchanged InitialSeedAccess::Upload. That function produces the
genuine seed on the owner stream, drains all D2D copies, and destroys its private
seed before returning. Only then will initialization allocate the paired
inventories/maintenance and incidence. Legacy cold initialization keeps its
existing order. Issuer attachment and publication remain last; any seed or
later workspace failure destroys the private transaction without attaching it.

The shared General preflight/initialize calculation becomes the maximum of:

- Complete steady transaction device payload.
- Primary transaction arena plus one producer's complete device peak.

Checked addition precedes the maximum. The primary arena is the existing
TransactionForecast.runtime_device_bytes; no formula is repeated in app code.
The immutable PreparedSource and the owning Plan/source upload remain alive
through both phases, so the existing conservative host coexistence bound stays
startup_host_bytes plus prepared retained_host_bytes. Both source preparation
and the physical owner are independently forecasted by their owners.

Owning tests retain exact two-slab/corner/identity comparisons, first physical
steps and active discard/retry. Exact device/host caps and one-byte-short
failure must exercise actual initialization. A large inventory-cap variant
exercises the steady-runtime-dominant maximum; the existing small runtime with
larger producer workspace exercises seed-overlap dominance. Legacy cold
forecast and allocation behavior remain covered by the original tests.
