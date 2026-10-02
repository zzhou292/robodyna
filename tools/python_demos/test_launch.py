"""Test launcher argument/environment routing, without substituting a physics test."""

import os
import unittest
from unittest import mock

from tools.python_demos import launch


class PythonDemoLauncher(unittest.TestCase):
    def test_original_arguments_follow_the_isolated_declared_interpreter(self):
        arguments = ["launcher", "--interpreter", "python", "--executor", "executor",
                     "--script", "script", "--package-manifest", "package", "--case", "case",
                     "--robodyna-data-root", "/data", "--robodyna-output-dir", "/run",
                     "--", "--help", "a b", "--physical-step", "0.002"]
        resolver = mock.Mock()
        resolver.Rlocation.side_effect = lambda name: "/declared/" + name
        with mock.patch.object(launch.sys, "argv", arguments), mock.patch.object(launch.runfiles, "Create", return_value=resolver), \
                mock.patch.object(launch.subprocess, "call", return_value=7) as call, \
                mock.patch.dict(os.environ, {"PYTHONPATH": "/ambient", "PYTHONHOME": "/other"}):
            with self.assertRaises(SystemExit) as result:
                launch.main()
        self.assertEqual(result.exception.code, 7)
        command = call.call_args.args[0]
        self.assertEqual(command[:5], ["/declared/python", "-I", "-B", "-X", "faulthandler"])
        self.assertEqual(command[-4:], ["--help", "a b", "--physical-step", "0.002"])
        self.assertNotIn("PYTHONPATH", call.call_args.kwargs["env"])
        self.assertNotIn("PYTHONHOME", call.call_args.kwargs["env"])


if __name__ == "__main__":
    unittest.main()
