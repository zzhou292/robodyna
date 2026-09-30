# Authenticated impact-response analysis

This focused offline module reads a closed physical run through
`output/physical_run::Replay`. It streams saved samples one at a time, reuses
`ParentPlasticMaxima`, and joins each runtime parent to the authenticated
original PID/material/section and source-node mapping. A bounded SAX reader adds
only static weak connectivity rows incident to yielded parents.

The report describes sampled native shell equivalent plastic strain, activity,
parent centroids, source attribution, and direct potential-transfer incidence.
It does not reconstruct stress, energy, reactions, continuous yield onset,
residual deformation, physical restart, self-contact, or constrained-DOF rank.
Centroid translation is motion context, not strain.

The CLI requires explicit external hashes for all three companion authorities
and creates a report outside the immutable run:

```text
robo_dyna_impact_response_analysis \
  VIEWER_INPUT VIEWER_SHA256 \
  RUN_SUMMARY RUN_SUMMARY_SHA256 \
  CONNECTIVITY_REPORT CONNECTIVITY_SHA256 \
  NEW_REPORT_JSON
```

Configure the standalone host-only project with the qualified Chrono package and
TL-FEA checkout:

```text
cmake -S analysis/impact_response -B BUILD \
  -DChrono_DIR=/path/to/Chrono/lib/cmake/Chrono \
  -DROBO_DYNA_TL_ROOT=/path/to/Total-Lagrangian-FEA
cmake --build BUILD --parallel 1 \
  --target robo_dyna_impact_response_analysis_check \
           robo_dyna_impact_response_analysis_cli
ctest --test-dir BUILD -R '^impact_response_analysis$' --output-on-failure
```

The synthetic gate covers 0/1/3/4-point layouts, unavailable versus
not-applicable fields, positive inactive history, sampled onset/peak/final
aggregation, exact yielded-shell connectivity binding, direct incident relation
classification, malformed connectivity rejection, and create-only report
publication outside the input run. The actual-run gate must additionally match
the authenticated run-summary extrema and counts.
