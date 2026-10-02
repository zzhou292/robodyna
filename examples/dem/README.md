# Retained DEM examples

These targets compile the original main functions directly against the native
Robodyna DEM owner. No placeholder executable or alternative solver is used.

| Root target | Retained source | Visualization |
| --- | --- | --- |
| `:ball_cosimulation` | `demo_DEM_ballCosim.cpp` | VSG |
| `:moving_boundary` | `demo_DEM_movingBoundary.cpp` | Original output-only path |
| `:fixed_terrain` | `demo_DEM_fixedTerrain.cpp` | Original output-only path |
| `:mixer` | `demo_DEM_mixer.cpp` | VSG |
| `:repose` | `demo_DEM_repose.cpp` | VSG |

`//examples/dem:native_demos` groups the five compiled programs. Building it does
not execute them. All programs retain their GPU dynamics, original JSON inputs,
unit conventions, time steps and simulation horizons. They are manual targets;
runtime qualification requires the existing workstation/GPU guards.

The module declares the original five JSON files and four directly referenced OBJ
meshes through `//src/dem:demo_assets`; VSG cases also carry the qualified renderer
assets. Original DEM arguments accept an optional JSON filename. They do not yet
provide the new product CLI's explicit asset/output-directory or bounded-run
interface. Their inherited data path defaults to `../data/`, so plain execution
from an arbitrary working directory is not a qualified launch procedure. A later
guarded runtime gate must provide the correct data directory and a fresh writable
output directory without writing into preserved source or evidence trees.

This batch establishes declared compilation targets. Successful compilation,
GPU execution and validated physical behavior must be recorded separately in the
demo inventory and receipts. See the [module contract](../../src/dem/README.md)
and [dependency plan](../../docs/migration/DEMO_DEPENDENCIES.md).
