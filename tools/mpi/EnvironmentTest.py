import unittest

from tools.mpi.environment import local_command, mpi_environment


class EnvironmentTest(unittest.TestCase):
    def test_ambient_remote_and_preload_configuration_is_removed(self):
        actual = mpi_environment({"OMPI_MCA_orte_default_hostfile": "/remote-hosts",
                                  "OMPI_MCA_plm": "rsh", "I_MPI_HYDRA_HOST_FILE": "other",
                                  "LD_PRELOAD": "other.so", "PYTHONHOME": "/other/python",
                                  "PATH": "/usr/bin"}, "/admitted/mpi")
        self.assertNotIn("OMPI_MCA_orte_default_hostfile", actual)
        self.assertNotIn("OMPI_MCA_plm", actual)
        self.assertNotIn("I_MPI_HYDRA_HOST_FILE", actual)
        self.assertNotIn("LD_PRELOAD", actual)
        self.assertNotIn("PYTHONHOME", actual)
        self.assertEqual(actual["PATH"], "/usr/bin")
        self.assertEqual(actual["OMP_THREAD_LIMIT"], "1")
        self.assertEqual(actual["CUDA_VISIBLE_DEVICES"], "")
        self.assertEqual(actual["OPAL_PREFIX"], "/admitted/mpi/prefix")

    def test_rank_bounds_reject_implicit_oversubscription(self):
        for value in (0, 9, -1, True, 1.5):
            with self.subTest(value=value), self.assertRaises(ValueError):
                local_command("mpi", "prefix", "program", value)

    def test_admitted_command_uses_only_local_shared_memory_ranks(self):
        command = local_command("/sdk/mpirun", "/sdk", "/test/program", 4)
        self.assertEqual(command[0], "/sdk/mpirun")
        self.assertEqual(command[-1], "/test/program")
        self.assertEqual(command[command.index("--host") + 1], "localhost:4")
        self.assertIn("isolated", command)
        self.assertIn("self,vader", command)
        self.assertIn("--nooversubscribe", command)


if __name__ == "__main__":
    unittest.main()
