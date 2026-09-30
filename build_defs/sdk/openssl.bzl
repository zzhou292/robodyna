"""Declare a local OpenSSL development SDK without downloading or installing it."""

def _first_file(ctx, candidates, description):
    for candidate in candidates:
        path = ctx.path(candidate)
        if path.exists and not path.is_dir:
            return path
    fail("Missing %s. Set ROBODYNA_OPENSSL_ROOT to the matching development SDK. Tried: %s" % (description, candidates))

def _system_openssl_impl(ctx):
    if ctx.os.name != "linux":
        fail("The current system OpenSSL provider supports Linux only; declare a provider for this platform.")
    root = ctx.os.environ.get("ROBODYNA_OPENSSL_ROOT", "/usr")
    if not root.startswith("/"):
        fail("ROBODYNA_OPENSSL_ROOT must be an absolute SDK root")
    triplet = {
        "amd64": "x86_64-linux-gnu",
        "x86_64": "x86_64-linux-gnu",
        "aarch64": "aarch64-linux-gnu",
    }.get(ctx.os.arch)
    header_dirs = [root + "/include/openssl"]
    library_dirs = [root + "/lib64", root + "/lib"]
    if triplet:
        header_dirs.append(root + "/include/" + triplet + "/openssl")
        library_dirs.append(root + "/lib/" + triplet)
    library = _first_file(ctx, [directory + "/libcrypto.so" for directory in library_dirs], "libcrypto.so")
    headers = {}
    for directory in header_dirs:
        path = ctx.path(directory)
        if not path.is_dir:
            continue
        ctx.watch(path)
        for header in path.readdir():
            if header.is_dir or not header.basename.endswith(".h"):
                continue
            if header.basename in headers:
                fail("Duplicate OpenSSL header %s in %s and %s" % (header.basename, headers[header.basename], header))
            headers[header.basename] = str(header)
            ctx.watch(header)
            ctx.symlink(header, "include/openssl/" + header.basename)
    for required in ["evp.h", "sha.h", "opensslv.h", "opensslconf.h"]:
        if required not in headers:
            fail("Missing OpenSSL header %s in declared SDK %s" % (required, root))
    version = "unknown"
    for line in ctx.read(headers["opensslv.h"]).splitlines():
        if "define OPENSSL_VERSION_TEXT " in line:
            version = line.split("OPENSSL_VERSION_TEXT", 1)[1].strip()
    if version == "unknown":
        fail("OpenSSL version macro is unavailable in %s" % headers["opensslv.h"])
    resolved_library = library.realpath
    ctx.watch(library)
    ctx.watch(resolved_library)
    output_library = "lib/" + resolved_library.basename
    ctx.symlink(resolved_library, output_library)
    ctx.file("sdk.json", json.encode_indent({
        "sdk_root": root,
        "header_version": version,
        "library_request": str(library),
        "library_resolved": str(resolved_library),
        "headers": headers,
        "scope": "Declared local SDK; runtime/header version parity requires the SDK test.",
    }) + "\n")
    ctx.file("BUILD.bazel", """load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")
package(default_visibility = ["//visibility:public"])
cc_import(name = "crypto_binary", shared_library = %s)
cc_library(name = "crypto", hdrs = glob(["include/openssl/*.h"]), includes = ["include"], deps = [":crypto_binary"])
exports_files(["sdk.json"])
""" % repr(output_library))

system_openssl = repository_rule(
    implementation = _system_openssl_impl,
    environ = ["ROBODYNA_OPENSSL_ROOT"],
    local = True,
    configure = True,
)
