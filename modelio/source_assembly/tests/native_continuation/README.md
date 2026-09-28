# Native tabulated source continuation

The authenticated OpenRadioss direct-import material adapter selects
`NativeLastSegment` for tabulated LAW44. The source curve's final knot does not
terminate the native constitutive law. This choice is independent of FAIL=0,
constant failure, and the selected native section integration.

The generic adapter does not change curves, stress units, rate settings,
failure thresholds, timestep, or material history. In particular supplied
positive C/P retain the Legacy rate policy and the resolved 10,000/s filter.
LAW1 and analytic LAW44 (including zero-C glass/midlayer declarations) retain
canonical StrictDomain metadata. Unknown hardening declarations reject.
Standalone TL preparation defaults remain StrictDomain.

The source authority is pinned OpenRadioss revision
`a62b27e6baa555d222a580d6218867d0be4d70b5`: SIGEPS44C calls VINTER; VINTER uses
the final interpolation segment beyond its final knot for ordinary FAIL=0 as
well as failure-enabled tables. TL's existing qualified NativeLastSegment
operator already implements that admission, including the separate native
EPSGM ceiling. This change selects it at the native source boundary.

The motivating frozen input is source element2235219/part2000141. Its third
layer tried to cross PLA .3 from .2999914066067876 and returned
CurveDomainExceeded under the incorrectly selected StrictDomain policy.
Production selection contains no element/part ID condition. The packet and
original failed vehicle run remain unchanged diagnostic evidence.

Owning tests cover native declaration selection, unchanged elastic/analytic/
standalone defaults, invalid tags, exact source-to-catalog propagation,
ordinary and failure policy separation, bit-identical in-domain NIP3 history/
work/rate updates, and actual device-resident parameter/curve/failure readback.
Existing source importer, analytic/mixed, complete vehicle resolution and
glass/midlayer/rigid field tests cover their existing boundaries. Independent
captured-operator and pinned Fortran comparisons remain separate receipts.
