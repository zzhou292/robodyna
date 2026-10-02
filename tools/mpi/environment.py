"""Local-only MPI test profile using the explicitly admitted OpenMPI runtime."""

import os
from pathlib import Path

from tools.bindings.run_swig import tool_environment


def mpi_environment(environment, sdk_directory):
    root = Path(sdk_directory).absolute()
    result = tool_environment(environment)
    for key in tuple(result):
        if key.startswith(("OMPI_", "OPAL_", "PMIX_", "PRTE_", "ORTE_", "I_MPI_", "MPICH_")):
            result.pop(key)
    for key in ("LD_PRELOAD", "LD_AUDIT", "LD_LIBRARY_PATH"):
        result.pop(key, None)
    result.update({
        "OPAL_PREFIX": str(root / "prefix"),
        "LD_LIBRARY_PATH": os.pathsep.join((str(root / "lib"), str(root / "prefix/lib/x86_64-linux-gnu"))),
        "OMPI_MCA_mca_base_component_path": str(root / "prefix/lib/x86_64-linux-gnu/openmpi/lib/openmpi3"),
        "OMP_NUM_THREADS": "1", "OMP_THREAD_LIMIT": "1", "OMP_DYNAMIC": "FALSE",
        "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1", "NUMEXPR_NUM_THREADS": "1",
        "CUDA_VISIBLE_DEVICES": "", "LC_ALL": "C",
    })
    return result


def local_command(mpirun, prefix, program, ranks):
    if isinstance(ranks, bool) or not isinstance(ranks, int) or not 1 <= ranks <= 8:
        raise ValueError("Local MPI tests admit one through eight explicitly bounded ranks")
    return [str(mpirun), "--prefix", str(prefix), "--host", "localhost:" + str(ranks),
            "--np", str(ranks), "--nooversubscribe", "--bind-to", "none",
            "--mca", "plm", "isolated", "--mca", "pml", "ob1",
            "--mca", "btl", "self,vader", "--mca", "oob_tcp_if_include", "lo",
            str(program)]
