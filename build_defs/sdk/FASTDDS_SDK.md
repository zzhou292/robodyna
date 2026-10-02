# Standalone distributed SDK

`ROBODYNA_FASTDDS_ROOT` identifies a workspace directory containing `fastdds/`,
`fastcdr/` and `foonathan/`. Their original source commits, build receipts,
generated configuration headers, libraries and installed CMake exports are pinned
by `fastdds_pins.json`. No provider downloads or installs packages.

The real exported interfaces determine the Bazel closure:

- FastDDS2.4.0 shared library and its405 public headers.
- FastCDR1.0.24 shared library and nine public headers.
- The original static PIC foonathan0.7.3 archive and45 public headers; its four
  published version/availability definitions are preserved.
- The SDK's actual OpenSSL SSL library and the existing `@openssl//:crypto`
  owner, plus Linux pthread/dl/rt linkage.

The Linux generator-expression branches were evaluated from the authenticated
CMake exports. Windows-only crypt32/iphlpapi/Shlwapi operands are not passed as
literal compiler/linker options. The original bundled Asio/TinyXML2 source pins
remain in the FastDDS build receipt; no substitute implementations are added.

This is the standalone profile required by the retained distributed module.
It deliberately does not reuse ROS2's different FastDDS2.6/FastCDR1.0.29 SDK.
No version macro is fabricated to hide an ABI mismatch. Original SDK licenses
are carried as runtime data. Library/provider admission is distinct from real
message serialization, MPI transport and DDS network qualification.
