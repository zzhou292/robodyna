# Owned QEPH activity-source certificate

The qualified eight-CTA production baseline is `1c8cf7e7`. Actual 101-step
measurement in the isolated `54472385` diagnostic found 307 complete scans of
324,095 QEPH parents, totaling 741,555,223 ns (mean 2.415489 ms). These include
accepted-witness and native contact activity queries; the aggregate is not an
exclusive accepted-witness stage total. All 49 saved payloads plus the manifest
matched the baseline exactly. No timer enters this production candidate.

`HostStorage::CheckActivitySectionSources` retains a successful immutable scan
for the exact actual `Collection()` pointer and `mixed_.get()` pointer. The
failure path's catalog belongs to the retained immutable failure binding, not
necessarily `collection_`. Prepared catalogs own their declarations/maps/curves
and reject reinitialization. HostStorage cannot be copied or assigned; normal
readback retains its existing single-owner contract. All three successful
collection/mixed publication sites invalidate the certificate before publishing
new backing. Failed setup leaves the old backing/certificate intact.

Every call still checks mixed/catalog and one-point availability and the actual
slab/count read shape. All mutable GPU flags, selected histories, role encoding,
force-cache validation, final role agreement, error priority, and output atomicity
remain unchanged. The certificate is not tied to an epoch or dynamic validity.
It adds two pointers (16 bytes on the qualified ABI) per HostStorage, no per-parent
allocation and no device memory. Existing sizeof-based host forecasts charge it;
full owning admission is still required before selecting the candidate.

The focused CMake reuses the existing QEPH mapped activity host/CUDA suites,
including prepared rollback, same-epoch corruption, alias, device-copy poison,
and earliest-error tests. New tests warm the certificate before each of the
three compact phase corruptions, retry at the same accepted epoch, exercise live
shape rejection, failed reinitialization, independent owners, and catalog lifetime
past the caller's destruction.

Run selected CTests `qeph_mapped_activity_host`, `qeph_mapped_activity_cuda`, and
`qeph_activity_source_cache_identity`. The old broad mapped-activity manifest is
already stale on untouched `1c8cf7e7` (first mismatch HostStorage.h); this focused
source checker instead pins the exact current qualified baseline and all four
changed production files. It additionally proves the original scan/readback and
all CUDA source remain unchanged, preserving frozen numerical readers in tests.
