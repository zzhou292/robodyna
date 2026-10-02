"""Compile the five original Vulkan RT shaders with the retained CMake flags."""

def _shaders_impl(ctx):
    outputs = []
    for source in ctx.files.srcs:
        output = ctx.actions.declare_file("vulkan/shaders/" + source.basename + ".spv")
        ctx.actions.run(
            executable = ctx.executable.compiler,
            arguments = ["-V", "--target-env", "vulkan1.2", "-o", output.path, source.path],
            inputs = [source],
            outputs = [output],
            mnemonic = "RobodynaSensorSpirv",
            progress_message = "Compile retained Sensor Vulkan shader " + source.basename,
        )
        outputs.append(output)
    return [DefaultInfo(files = depset(outputs), runfiles = ctx.runfiles(files = outputs))]

sensor_shaders = rule(
    implementation = _shaders_impl,
    attrs = {
        "srcs": attr.label_list(mandatory = True, allow_files = True),
        "compiler": attr.label(default = Label("@sensor_sdk//:bin/glslangValidator"), executable = True, cfg = "exec", allow_single_file = True),
    },
)
