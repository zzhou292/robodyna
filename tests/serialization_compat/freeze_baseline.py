"""Freeze successfully generated, pre-rename archives without replacing evidence."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


ARCHIVES = ("baseline.json", "baseline.xml", "baseline.bin", "producer.json")
CHRONO = "src/compatibility/chrono/src/chrono/"
# These pin the relevant current archive/type definitions, not an assertion that
# unrelated working-tree changes or every transitive source remained unchanged.
PRODUCTION_INPUTS = (
    "core/ChClassFactory.h", "core/ChClassFactory.cpp",
    "core/ChVector3.h", "core/ChQuaternion.h", "core/ChFrame.h", "core/ChFrameMoving.h",
    "serialization/ChArchive.h", "serialization/ChArchive.cpp",
    "serialization/ChArchiveJSON.h", "serialization/ChArchiveJSON.cpp",
    "serialization/ChArchiveXML.h", "serialization/ChArchiveXML.cpp",
    "serialization/ChArchiveBinary.h", "serialization/ChArchiveBinary.cpp",
    "physics/ChBody.h", "physics/ChBody.cpp", "physics/ChBodyAuxRef.h", "physics/ChBodyAuxRef.cpp",
    "physics/ChBodyFrame.h", "physics/ChBodyFrame.cpp",
    "physics/ChObject.h", "physics/ChObject.cpp", "physics/ChPhysicsItem.h", "physics/ChPhysicsItem.cpp",
)
PRODUCER_INPUTS = (
    "ArchiveFormats.h", "ArchiveFormats.cpp", "FixtureModel.h", "FixtureModel.cpp", "WriteBaseline.cpp", "BUILD.bazel",
)

# This new checkpoint uses the actual post-Body-move locations. Keep the older
# profiles above tied to their historical source layout; never silently retarget
# or regenerate their frozen fixtures after a rename.
SYSTEM_PRODUCTION_INPUTS = (
    "src/compatibility/chrono/src/chrono/core/ChClassFactory.h",
    "src/compatibility/chrono/src/chrono/core/ChClassFactory.cpp",
    "src/compatibility/chrono/src/chrono/core/ChVector3.h",
    "src/compatibility/chrono/src/chrono/core/ChQuaternion.h",
    "src/compatibility/chrono/src/chrono/core/ChFrame.h",
    "src/compatibility/chrono/src/chrono/core/ChFrameMoving.h",
    "src/compatibility/chrono/src/chrono/serialization/ChArchive.h",
    "src/compatibility/chrono/src/chrono/serialization/ChArchive.cpp",
    "src/compatibility/chrono/src/chrono/serialization/ChArchiveJSON.h",
    "src/compatibility/chrono/src/chrono/serialization/ChArchiveJSON.cpp",
    "src/compatibility/chrono/src/chrono/serialization/ChArchiveXML.h",
    "src/compatibility/chrono/src/chrono/serialization/ChArchiveXML.cpp",
    "src/compatibility/chrono/src/chrono/serialization/ChArchiveBinary.h",
    "src/compatibility/chrono/src/chrono/serialization/ChArchiveBinary.cpp",
    "include/robodyna/mbd/RbBody.h",
    "src/mbd/bodies/RbBody.cpp",
    "include/robodyna/mbd/RbBodyFwd.h",
    "src/compatibility/chrono/src/chrono/physics/ChBody.h",
    "src/compatibility/chrono/src/chrono/physics/ChBodyFwd.h",
    "src/compatibility/chrono/src/chrono/physics/ChBodyAuxRef.h",
    "src/compatibility/chrono/src/chrono/physics/ChBodyAuxRef.cpp",
    "src/compatibility/chrono/src/chrono/physics/ChBodyFrame.h",
    "src/compatibility/chrono/src/chrono/physics/ChBodyFrame.cpp",
    "src/compatibility/chrono/src/chrono/physics/ChObject.h",
    "src/compatibility/chrono/src/chrono/physics/ChObject.cpp",
    "src/compatibility/chrono/src/chrono/physics/ChPhysicsItem.h",
    "src/compatibility/chrono/src/chrono/physics/ChPhysicsItem.cpp",
    "src/compatibility/chrono/src/chrono/physics/ChSystem.h",
    "src/compatibility/chrono/src/chrono/physics/ChSystem.cpp",
    "src/compatibility/chrono/src/chrono/physics/ChSystemNSC.h",
    "src/compatibility/chrono/src/chrono/physics/ChSystemNSC.cpp",
    "src/compatibility/chrono/src/chrono/physics/ChSystemSMC.h",
    "src/compatibility/chrono/src/chrono/physics/ChSystemSMC.cpp",
    "src/compatibility/chrono/src/chrono/physics/ChAssembly.h",
    "src/compatibility/chrono/src/chrono/physics/ChAssembly.cpp",
    "src/compatibility/chrono/src/chrono/physics/ChContactContainerNSC.h",
    "src/compatibility/chrono/src/chrono/physics/ChContactContainerNSC.cpp",
    "src/compatibility/chrono/src/chrono/physics/ChContactContainerSMC.h",
    "src/compatibility/chrono/src/chrono/physics/ChContactContainerSMC.cpp",
    "src/compatibility/chrono/src/chrono/solver/ChSystemDescriptor.h",
    "src/compatibility/chrono/src/chrono/solver/ChSolverPSOR.h",
    "src/compatibility/chrono/src/chrono/solver/ChSolverPSOR.cpp",
    "src/compatibility/chrono/src/chrono/physics/ChContactMaterial.h",
    "src/compatibility/chrono/src/chrono/physics/ChContactMaterial.cpp",
)
SYSTEM_PRODUCER_INPUTS = (
    "tests/system_compat/SystemFixture.h", "tests/system_compat/SystemFixture.cpp",
    "tests/system_compat/WriteBaseline.cpp", "tests/system_compat/BUILD.bazel",
    "tests/archive_support/StreamArchive.h", "tests/archive_support/BUILD.bazel",
)


def production_paths(profile):
    if profile == "system":
        return SYSTEM_PRODUCTION_INPUTS
    inputs = PRODUCTION_INPUTS
    if profile == "body":
        inputs += ("physics/ChBodyEasy.h", "physics/ChBodyEasy.cpp",
                   "physics/ChMarker.h", "physics/ChMarker.cpp",
                   "physics/ChForce.h", "physics/ChForce.cpp")
    return tuple(CHRONO + path for path in inputs)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def file_record(path):
    data = path.read_bytes()
    return {"bytes": len(data), "sha256": digest(data)}


def freeze(repo, generated, destination, binary, profile):
    if destination.exists():
        raise ValueError("fixture destination already exists; refusing overwrite")
    baseline = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
    production = {}
    for relative in production_paths(profile):
        actual = (repo / relative).read_bytes()
        committed = subprocess.check_output(["git", "show", f"{baseline}:{relative}"], cwd=repo)
        if actual != committed:
            raise ValueError(f"pre-rename production input changed from recorded HEAD: {relative}")
        production[relative] = {"sha256": digest(actual), "matches_recorded_head": True}
    files = {name: file_record(generated / name) for name in ARCHIVES}
    producer_metadata = json.loads((generated / "producer.json").read_text())
    if not producer_metadata.get("compiler") or producer_metadata.get("pointer_bytes") != 8:
        raise ValueError("producer metadata does not identify the qualified 64-bit toolchain")
    tests = repo / "tests/serialization_compat"
    producer_inputs = PRODUCER_INPUTS
    if profile == "rename-probe":
        tests = tests / "rename_probe"
        producer_inputs = ("LegacyTypes.h", "LegacyTypes.cpp", "WriteLegacyProbe.cpp", "BUILD.bazel")
    if profile == "body":
        tests = repo
        producer_inputs = ("tests/body_compat/BodyFixture.h", "tests/body_compat/BodyFixture.cpp",
                           "tests/body_compat/WriteBaseline.cpp", "tests/body_compat/BUILD.bazel",
                           "tests/archive_support/StreamArchive.h", "tests/archive_support/BUILD.bazel")
    if profile == "system":
        tests = repo
        producer_inputs = SYSTEM_PRODUCER_INPUTS
    manifest = {
        "schema": "robodyna.serialization-compatibility-baseline.v1",
        "baseline_head": baseline,
        "profile": profile,
        "scope": "Pre-rename object archives; individual archive/type inputs match HEAD. This is not a physical restart or whole-tree cleanliness claim.",
        "platform": "Linux x86_64 GCC/Itanium ABI; native archive sizes/version identities are toolchain-specific",
        "files": files,
        "producer_binary": file_record(binary),
        "producer_inputs": {str((tests / name).relative_to(repo)): file_record(tests / name) for name in producer_inputs},
        "production_inputs": production,
        "observed_producer": producer_metadata,
    }
    destination.mkdir()
    for name in ARCHIVES:
        shutil.copyfile(generated / name, destination / name)
    (destination / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"Frozen {len(files)} files at {destination}; production archive inputs match {baseline}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--generated-dir", type=Path, required=True)
    parser.add_argument("--destination", type=Path, required=True)
    parser.add_argument("--producer-binary", type=Path, required=True)
    parser.add_argument("--profile", choices=("actual-types", "rename-probe", "body", "system"), default="actual-types")
    args = parser.parse_args()
    freeze(args.repo_root.resolve(), args.generated_dir.resolve(), args.destination.resolve(), args.producer_binary.resolve(), args.profile)
