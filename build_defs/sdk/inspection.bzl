"""Small shared helpers for explicit local native SDK admission."""

def required_file(ctx, value):
    path = ctx.path(value)
    if not path.exists or path.is_dir:
        fail("Missing declared SDK file: " + value)
    ctx.watch(path)
    ctx.watch(path.realpath)
    return path

def run_tool(ctx, name, arguments):
    tool = ctx.which(name)
    if tool == None:
        fail("Native SDK inspection needs host tool " + name)
    result = ctx.execute([str(tool)] + arguments, quiet = True)
    if result.return_code:
        fail("SDK inspection failed: %s: %s" % (name, result.stderr))
    return result.stdout

def file_sha256(ctx, path):
    return run_tool(ctx, "sha256sum", [str(path)]).split(" ")[0]

def verify_file_hashes(ctx, root, pins):
    """Authenticate a declared relative-path/hash map in bounded tool batches."""
    paths = {name: required_file(ctx, root + "/" + name) for name in pins}
    names = paths.keys()
    for start in range(0, len(names), 256):
        selected = names[start:start + 256]
        output = run_tool(ctx, "sha256sum", [str(paths[name].realpath) for name in selected]).splitlines()
        if len(output) != len(selected):
            fail("SDK checksum output is incomplete")
        for index, line in enumerate(output):
            name = selected[index]
            if line.split(" ")[0] != pins[name]:
                fail("SDK file differs from its admitted pin: " + name)
    return paths

def shared_library(ctx, requested, soname):
    path = required_file(ctx, requested)
    dynamic = run_tool(ctx, "readelf", ["-d", str(path.realpath)])
    actual = None
    needed = []
    for line in dynamic.splitlines():
        if "(SONAME)" in line:
            actual = line.split("[", 1)[1].split("]", 1)[0]
        elif "(NEEDED)" in line:
            needed.append(line.split("[", 1)[1].split("]", 1)[0])
    if actual != soname:
        fail("SDK SONAME mismatch: expected %s, got %s" % (soname, actual))
    if "Chrono" in soname or any(["Chrono" in item for item in needed]):
        fail("External SDK cannot supply an inherited physics implementation")
    output = "lib/" + soname
    ctx.symlink(path.realpath, output)
    return struct(output = output, record = {
        "requested": requested,
        "resolved": str(path.realpath),
        "sha256": file_sha256(ctx, path.realpath),
        "soname": soname,
        "needed": needed,
    })
