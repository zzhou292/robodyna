# Native parser dependencies

The parser libraries compile the retained source implementations. The root Bazel
repository providers declare their external dependencies; none imports another
Robodyna/Chrono mechanics library.

| Repository | Actual dependency | Explicit environment |
| --- | --- | --- |
| `@python_embed_sdk` | Linux CPython 3.10 embedding DSO | `ROBODYNA_PYTHON_EXECUTABLE`, identical to `@python_sdk` |
| `@urdf_sdk` | URDFDOM 4.0.1 | `ROBODYNA_URDF_ROOT` |
| `@urdf_sdk` | URDFDOM headers 1.1.2 | `ROBODYNA_URDF_HEADERS_ROOT` |
| `@urdf_sdk` | console_bridge 1.0.2 | `ROBODYNA_CONSOLE_BRIDGE_ROOT` |
| `@urdf_sdk` | TinyXML2 11.0.0 | `ROBODYNA_TINYXML2_ROOT` |

The four URDF inputs are pinned official source commits and built into separate,
create-only workspace SDK prefixes. `tools/dependencies/cmake_sdk.py` authenticates
archives, records CMake settings and installation files, and verifies that the
build did not change the source tree. Admission then checks each receipt, public
header sentinel, file hash, actual DSO SONAME and dependency. License copies come
from those exact source archives and are retained in `urdf_licenses/` with their
source-member identities. No host package installation is performed.

SDK source revisions:

- TinyXML2: `9148bdf719e997d1f474be6bcc7943881046dba1`.
- console_bridge: `0828d846f2d4940b4e2b5075c6c724991d0cd308`.
- URDFDOM headers: `7b1b3a44509985b953526dcd1fffacd3b2f92615`.
- URDFDOM: `3f6bf9a608065464bb6c2828af6e96c8f577f2bf`.

CPython headers continue to come from the existing `@python_sdk`. The embedding
provider locates and records `libpython3.10.so.1.0` through that same interpreter's
configuration, then admits its real loader dependencies. The original Python
parser owns interpreter initialization/finalization, with one interpreter alive
at a time. The SolidWorks example imports the declared binding package and links
the same `librobodyna_core.so` as its extension; it must not load an unrelated
installed `pychrono` package.

Source admission, successful compilation and runtime qualification are separate
results. Required native gates are the actual URDF parser/body/joint test and the
embedded Python import/finalize/shared-body step test. Their target and receipt
status lives with the parser integration, not in this SDK version table.
