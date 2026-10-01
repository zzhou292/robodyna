"""Scoped real-project SWIG configuration, generation and install qualification."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess


DISABLED_MODULES = ("CASCADE", "DEM", "FMI", "FSI", "IRRLICHT", "MODAL", "MULTICORE", "MUMPS",
                    "PARDISO_MKL", "PARSERS", "PERIDYNAMICS", "POSTPROCESS", "PRECICE",
                    "ROS", "SENSOR", "SYNCHRONO", "VEHICLE", "VEHICLE_MODELS", "VSG")


def run(command, log, env, succeeds=True, cwd=None):
    result = subprocess.run(command, env=env, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    with log.open("x") as stream:
        stream.write(result.stdout)
    if (result.returncode == 0) is not succeeds:
        raise RuntimeError(f"Unexpected command status {result.returncode}; see {log}")
    return result.stdout


def real_project(repo, output, sdk, eigen, ninja, python_enabled):
    output.mkdir()
    build, prefix = output / "build", output / "install"
    env = dict(os.environ, SWIG_LIB=str(sdk / "usr/share/swig4.0"))
    command = ["/usr/bin/cmake", "-S", str(repo / "src/compatibility/chrono"), "-B", str(build), "-GNinja",
               "-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_MAKE_PROGRAM=" + str(ninja), "-DCMAKE_INSTALL_PREFIX=" + str(prefix),
               "-DBUILD_DEMOS=OFF", "-DBUILD_TESTING=OFF", "-DBUILD_BENCHMARKING=OFF",
               "-DBUILD_SHARED_LIBS=ON", "-DCH_ENABLE_MODULE_FEA=ON", "-DCH_ENABLE_MODULE_FEA_MULTIPHYSICS=OFF",
               "-DCH_ENABLE_MODULE_PYTHON=" + ("ON" if python_enabled else "OFF"),
               "-DCH_ENABLE_MODULE_CSHARP=ON", "-DCH_INSTALL_SWIG_INTERFACE=ON",
               # Retained C# CMake declares its robot wrapper unconditionally.
               "-DCH_ENABLE_MODULE_ROBOT_MODELS=ON",
               "-DCHRONO_GPU_VENDOR=NONE", "-DCH_ENABLE_OPENMP=OFF", "-DCH_USE_SIMD=OFF",
               "-DCH_ENABLE_HDF5=OFF", "-DCH_ENABLE_YAML=OFF", "-DUSE_CCACHE=OFF",
               "-DCMAKE_DISABLE_FIND_PACKAGE_Thrust=ON", "-DCMAKE_DISABLE_FIND_PACKAGE_OpenMP=ON",
               "-DPython3_EXECUTABLE=/usr/bin/python3.10", "-DEIGEN3_INCLUDE_DIR=" + str(eigen),
               "-DSWIG_EXECUTABLE=" + str(sdk / "usr/bin/swig4.0")]
    command += ["-DCH_ENABLE_MODULE_" + module + "=OFF" for module in DISABLED_MODULES]
    run(command, output / "configure.log", env)
    # No -DROBODYNA_SOURCE_ROOT is supplied: the live sibling-scope resolver is exercised.
    run(["/usr/bin/cmake", "--build", str(build), "--target", "robodyna_swig_install_inputs", "--parallel", "1"],
        output / "declarations.log", env)
    targets = run([str(ninja), "-C", str(build), "-t", "targets", "all"], output / "targets.log", env)
    wrapper_targets = []
    numpy = False
    for language, suffix in (("python", "ChModuleCore_pythonPYTHON_wrap.cxx"),
                             ("csharp", "ChModuleCore_csharpCSHARP_wrap.cxx")):
        if language == "python" and not python_enabled:
            continue
        matching = [line.rsplit(": ", 1)[0] for line in targets.splitlines() if line.rsplit(": ", 1)[0].endswith(suffix)]
        # Ninja publishes relative and absolute aliases for the same output.
        resolved = {str((build / name).resolve()) for name in matching}
        if len(resolved) != 1:
            raise RuntimeError(f"Expected one actual configured {language} wrapper output: {matching}")
        target = matching[0]
        commands = run([str(ninja), "-C", str(build), "-t", "commands", target], output / (language + "-commands.log"), env)
        swig_lines = [line for line in commands.splitlines() if "swig4.0" in line and " -" + language in line]
        if len(swig_lines) != 1 or "-I" + str(repo / "include") not in swig_lines[0] or "-I" + str(build) not in swig_lines[0]:
            raise RuntimeError("Configured SWIG command lacks canonical/generated include directories")
        numpy = numpy or "-DCHRONO_PYTHON_NUMPY" in swig_lines[0]
        edges = [line for line in (build / "build.ninja").read_text().splitlines()
                 if line.startswith("build ") and target in line and ": CUSTOM_COMMAND" in line]
        if len(edges) != 1 or "robodyna_swig/BodyDeclarations.h" not in edges[0]:
            raise RuntimeError("Configured SWIG action lacks its declared generated-header dependency")
        # The retained target has an order-only dependency on the whole native
        # core. Execute its exact emitted SWIG command after its declared view
        # inputs; do not claim this is a complete Ninja target/backend build.
        if " -c " in swig_lines[0] and any(tool in swig_lines[0] for tool in ("/c++", "/g++", "/cc ")):
            raise RuntimeError("Unexpected native compilation in the selected SWIG command")
        run(["/bin/sh", "-c", swig_lines[0]], output / (language + "-generation.log"), env, cwd=build)
        wrapper_targets.append(target)
    # Execute only this owning install script; child library installation is deliberately excluded.
    script = build / "src/chrono_swig/cmake_install.cmake"
    run(["/usr/bin/cmake", "-DCMAKE_INSTALL_PREFIX=" + str(prefix), "-DCMAKE_INSTALL_LOCAL_ONLY=1", "-P", str(script)],
        output / "install.log", env)
    for name in ("BodyDeclarations.h", "BodyDeclarations.h.json"):
        source = build / "robodyna_swig" / name
        installed = prefix / "include/robodyna_swig" / name
        if not installed.is_file() or installed.read_bytes() != source.read_bytes():
            raise RuntimeError("Installed declaration view or receipt differs from generated input")
    if not (prefix / "include/chrono_swig/core/ChBody.i").is_file():
        raise RuntimeError("Retained body interface was not installed")
    return {"python_enabled": python_enabled, "source_root_supplied_on_command_line": False,
            "retained_cmake_detected_numpy": numpy,
            "robot_models_declared_for_existing_csharp_dependency": True,
            "wrapper_targets": wrapper_targets, "configured_generation_passed": True,
            "execution_scope": "Exact configured SWIG commands after generated dependencies; full Ninja target not built",
            "owning_interface_install_passed": True, "native_backend_compiled": False}


def contract_regeneration(repo, output, ninja):
    output.mkdir()
    fixture, build = output / "source", output / "build"
    fixture.mkdir()
    for name in ("tools/bindings/declaration_view.py", "tools/migration/source_transform.py",
                 "docs/migration/BODY_TRANSFORMATIONS.json", "include/robodyna/mbd/RbBody.h",
                 "src/compatibility/chrono/src/chrono/physics/ChBody.h"):
        target = fixture / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.symlink_to(repo / name)
    contract = fixture / "build_defs/bindings/body_view_contract.json"
    contract.parent.mkdir(parents=True)
    original = (repo / "build_defs/bindings/body_view_contract.json").read_bytes()
    contract.write_bytes(original)
    (fixture / "CMakeLists.txt").write_text(
        'cmake_minimum_required(VERSION 3.22)\nproject(BodyContractRegeneration NONE)\n'
        f'set(ROBODYNA_SOURCE_ROOT "{fixture}")\n'
        f'include("{repo}/build_defs/bindings/declaration_view.cmake")\n'
        'add_custom_target(binding_probe)\nrobodyna_bind_swig_body(binding_probe)\n')
    env = dict(os.environ)
    run(["/usr/bin/cmake", "-S", str(fixture), "-B", str(build), "-GNinja", "-DCMAKE_MAKE_PROGRAM=" + str(ninja)], output / "configure.log", env)
    command = ["/usr/bin/cmake", "--build", str(build), "--target", "binding_probe", "--parallel", "1"]
    run(command, output / "initial.log", env)
    corrupt = json.loads(original)
    corrupt["expected_ledger_sha256"] = "0" * 64
    contract.write_text(json.dumps(corrupt))
    error = run(command, output / "bad-contract.log", env, succeeds=False)
    if "reviewed ledger pin" not in error or (build / "robodyna_swig/BodyDeclarations.h").exists():
        raise RuntimeError("CMake retained stale configured pins or stale generated output")
    contract.write_bytes(original)
    run(command, output / "recovered.log", env)
    return {"copied_contract_only": True, "changed_pin_reconfigured_and_rejected": True, "restored_pin_recovered": True}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sdk", type=Path, required=True)
    parser.add_argument("--eigen", type=Path, required=True)
    parser.add_argument("--ninja", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir()
    results = [real_project(args.repo, args.output / "both", args.sdk, args.eigen, args.ninja, True),
               real_project(args.repo, args.output / "csharp-only", args.sdk, args.eigen, args.ninja, False)]
    regeneration = contract_regeneration(args.repo, args.output / "regeneration", args.ninja)
    (args.output / "receipt.json").write_text(json.dumps({"schema": "robodyna.cmake_swig_boundary.v1", "passed": True,
                                                        "profiles": results, "regeneration": regeneration}, indent=2) + "\n")
    print("Configured SWIG commands, owning interface installation and contract regeneration passed")
