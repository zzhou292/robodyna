# Native beam contact property area

The new value entry point returns DEFBEAM_SECT ISECT2 AREA, stored into GEO1 by
HM_READ_PROP18 before interface initialization. PMASS later overwrites GEO1 with
the integration-point sum; that existing Section.area field remains unchanged.
The shared helper preserves the original PI and multiplication association, with
no Reference layout, identity, section-point, mass or force changes.

The independent wrapper reuses the existing complete native DEFBEAM_SECT library
and copies the original reader print/store blocks plus the selected I25STI3 beam
half-sqrt expression. Its four channels are AREA, reader GEO1, AREA_I and raw beam
gap. The old post-PMASS oracle and all existing reference tests remain controls.
Native arrays are expected values only, never source operands in production.

Three new host groups cover native working spaces/neighboring radii, invalid or
unrepresentable results with preserved output/retry, and area into the qualified
gap value controller. The sampled AREA/AREA_I difference count is recorded rather
than assuming they differ for a chosen radius. Enable BEAM18_CUDA for the existing
actual device reference regression because PrepareSection shares the helper.
No GPU hot-path or full-source binding claim follows from this startup leaf.
