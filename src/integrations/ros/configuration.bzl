"""Retained ROS configuration, using an explicit launch-directory node binding."""

def _configuration_impl(ctx):
    header = ctx.actions.declare_file("generated/chrono_ros/ChConfigROS.h")
    availability = '#include "chrono/ChConfig.h"'
    if ctx.attr.sensor:
        availability += '\n#define CHRONO_SENSOR\n#include "chrono_sensor/ChConfigSensor.h"'
    if ctx.attr.urdf:
        availability += '\n#define CHRONO_HAS_URDF'
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = header,
        substitutions = {
            '#include "chrono/ChConfig.h"': availability,
            # The admitted launcher places a link to the declared real node in
            # its create-only working directory. No workstation path is compiled
            # into the module, and the old binary-name contract stays intact.
            "@CH_ROS_NODE_BUILD_PATH@": "chrono_ros_node",
            "@CH_ROS_NODE_INSTALL_PATH@": "",
        },
    )
    return [DefaultInfo(files = depset([header]))]

ros_configuration = rule(
    implementation = _configuration_impl,
    attrs = {"template": attr.label(allow_single_file = True, mandatory = True),
             "sensor": attr.bool(default = False), "urdf": attr.bool(default = False)},
)

def _runfile_path(ctx, file):
    if file.short_path.startswith("../"):
        return file.short_path[3:]
    return ctx.workspace_name + "/" + file.short_path

def _node_paths_impl(ctx):
    output = ctx.actions.declare_file("NodePaths.py")
    ctx.actions.write(
        output,
        "# Declared runfile identities; no workstation paths.\n" +
        "BACKEND = " + repr(_runfile_path(ctx, ctx.file.backend)) + "\n" +
        "SDK_RECEIPT = " + repr(_runfile_path(ctx, ctx.file.sdk_receipt)) + "\n" +
        "MESSAGE_SDK_RECEIPTS = " + repr([_runfile_path(ctx, file) for file in ctx.files.message_sdk_receipts]) + "\n",
    )
    return [DefaultInfo(files = depset([output]))]

node_paths = rule(
    implementation = _node_paths_impl,
    attrs = {
        "backend": attr.label(allow_single_file = True, mandatory = True),
        "sdk_receipt": attr.label(allow_single_file = True, mandatory = True),
        "message_sdk_receipts": attr.label_list(allow_files = True),
    },
)
