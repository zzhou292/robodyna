"""Exact native source/header ownership for the first neutral mechanics seam."""

NEUTRAL_SOURCES = {
    "foundation": [
        "src/chrono/core/ChClassFactory.cpp",
        "src/chrono/core/ChQuaternion.cpp",
        "src/chrono/core/ChRotation.cpp",
        "src/chrono/core/ChVector3.cpp",
        "src/chrono/core/ChCoordsys.cpp",
        "src/chrono/core/ChQuadrature.cpp",
        "src/chrono/core/ChBezierCurve.cpp",
        "src/chrono/core/ChCubicSpline.cpp",
        "src/chrono/core/ChRandom.cpp",
        "src/chrono/core/ChDataPath.cpp",
        "src/chrono/serialization/ChArchive.cpp",
        "src/chrono/serialization/ChArchiveBinary.cpp",
        "src/chrono/serialization/ChObjectExplorer.cpp",
        "src/chrono/serialization/ChArchiveJSON.cpp",
        "src/chrono/serialization/ChArchiveXML.cpp",
        "src/chrono/serialization/ChArchiveASCII.cpp"
    ],
    "mass_blocks": [
        "src/chrono/solver/ChVariables.cpp",
        "src/chrono/solver/ChVariablesBody.cpp",
        "src/chrono/solver/ChVariablesBodyOwnMass.cpp"
    ],
    "frames": [
        "src/chrono/physics/ChBodyFrame.cpp"
    ],
    "inertia": [
        "src/chrono/physics/ChMassProperties.cpp"
    ]
}

NEUTRAL_HEADERS = {
    "mass_blocks": [
        "src/chrono/solver/ChVariables.h",
        "src/chrono/solver/ChVariablesBody.h",
        "src/chrono/solver/ChVariablesBodyOwnMass.h"
    ],
    "frames": [
        "src/chrono/physics/ChBodyFrame.h"
    ],
    "inertia": [
        "src/chrono/physics/ChMassProperties.h"
    ],
    "foundation": [
        "src/chrono/ChCorePCH.h",
        "src/chrono/core/ChApiCE.h",
        "src/chrono/core/ChBezierCurve.h",
        "src/chrono/core/ChClassFactory.h",
        "src/chrono/core/ChCoordsys.h",
        "src/chrono/core/ChCubicSpline.h",
        "src/chrono/core/ChDataPath.h",
        "src/chrono/core/ChFrame.h",
        "src/chrono/core/ChFrameMoving.h",
        "src/chrono/core/ChMatrix.h",
        "src/chrono/core/ChMatrix33.h",
        "src/chrono/core/ChMatrixEigenExtensions.h",
        "src/chrono/core/ChMatrixMBD.h",
        "src/chrono/core/ChPlatform.h",
        "src/chrono/core/ChQuadrature.h",
        "src/chrono/core/ChQuaternion.h",
        "src/chrono/core/ChRandom.h",
        "src/chrono/core/ChRotation.h",
        "src/chrono/core/ChSparseMatrixEigenExtensions.h",
        "src/chrono/core/ChTemplateExpressions.h",
        "src/chrono/core/ChTypes.h",
        "src/chrono/core/ChVector3.h",
        "src/chrono/serialization/ChArchive.h",
        "src/chrono/serialization/ChArchiveASCII.h",
        "src/chrono/serialization/ChArchiveBinary.h",
        "src/chrono/serialization/ChArchiveJSON.h",
        "src/chrono/serialization/ChArchiveXML.h",
        "src/chrono/serialization/ChObjectExplorer.h",
        "src/chrono/utils/ChConstants.h",
        "src/chrono/utils/ChUtils.h",
        "src/chrono_thirdparty/rapidjson/allocators.h",
        "src/chrono_thirdparty/rapidjson/document.h",
        "src/chrono_thirdparty/rapidjson/encodedstream.h",
        "src/chrono_thirdparty/rapidjson/encodings.h",
        "src/chrono_thirdparty/rapidjson/error/error.h",
        "src/chrono_thirdparty/rapidjson/filereadstream.h",
        "src/chrono_thirdparty/rapidjson/filewritestream.h",
        "src/chrono_thirdparty/rapidjson/internal/biginteger.h",
        "src/chrono_thirdparty/rapidjson/internal/clzll.h",
        "src/chrono_thirdparty/rapidjson/internal/diyfp.h",
        "src/chrono_thirdparty/rapidjson/internal/dtoa.h",
        "src/chrono_thirdparty/rapidjson/internal/ieee754.h",
        "src/chrono_thirdparty/rapidjson/internal/itoa.h",
        "src/chrono_thirdparty/rapidjson/internal/meta.h",
        "src/chrono_thirdparty/rapidjson/internal/pow10.h",
        "src/chrono_thirdparty/rapidjson/internal/stack.h",
        "src/chrono_thirdparty/rapidjson/internal/strfunc.h",
        "src/chrono_thirdparty/rapidjson/internal/strtod.h",
        "src/chrono_thirdparty/rapidjson/internal/swap.h",
        "src/chrono_thirdparty/rapidjson/memorystream.h",
        "src/chrono_thirdparty/rapidjson/msinttypes/inttypes.h",
        "src/chrono_thirdparty/rapidjson/msinttypes/stdint.h",
        "src/chrono_thirdparty/rapidjson/prettywriter.h",
        "src/chrono_thirdparty/rapidjson/rapidjson.h",
        "src/chrono_thirdparty/rapidjson/reader.h",
        "src/chrono_thirdparty/rapidjson/stream.h",
        "src/chrono_thirdparty/rapidjson/stringbuffer.h",
        "src/chrono_thirdparty/rapidjson/writer.h",
        "src/chrono_thirdparty/rapidxml/rapidxml.hpp"
    ]
}

NEUTRAL_TARGETS = {
    "foundation": "//src/core:foundation",
    "mass_blocks": "//src/numerics/variables:mass_blocks",
    "frames": "//src/mechanics/kinematics:frames",
    "inertia": "//src/mechanics/inertia:inertia"
}

# Original identities above remain pinned to the immutable source import.
# Source locations are mapped independently in source_paths.bzl.
NEUTRAL_CANONICAL_HEADERS = {
    "inertia": ["include/robodyna/mechanics/RbMassProperties.h"],
}

NEUTRAL_HEADER_TARGETS = {
    "inertia": ["//include/robodyna/mechanics:inertia_headers"],
}
