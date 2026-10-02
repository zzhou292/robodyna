# Scene exporters

`//src/postprocess:exporters` exposes the retained POV-Ray and Blender exporters
through one native Bazel owner. The original two CMake translation units compile
directly and share the existing mechanics backend; no second core library or
simulation owner is introduced.

The Gnuplot helper is header-only. Compiling its demos does not prove that an
external Gnuplot executable is present or qualify plot execution. Runtime tool,
data and writable-output admission remain explicit requirements. The inherited
public headers/names stay available during the staged Robodyna API migration.
