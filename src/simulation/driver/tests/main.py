import unittest

from src.simulation.driver.tests import test_admission, test_launch, test_rendering, test_guard_runtime, test_viewer_assets

if __name__ == "__main__":
    suite = unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromModule(module)
                               for module in (test_admission, test_launch, test_rendering, test_guard_runtime, test_viewer_assets))
    raise SystemExit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
