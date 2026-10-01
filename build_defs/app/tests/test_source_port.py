"""Check literal-source admission failures without loading a compiler."""

import unittest

from build_defs.app.cmake_sources import library_source


class LiteralCmakeSourceTest(unittest.TestCase):
    def test_additive_sources_and_private_dependencies(self):
        source = '''add_library(run STATIC "${CMAKE_CURRENT_LIST_DIR}/Run.cpp")
target_sources(run PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../Shared.cpp")
target_link_libraries(run PUBLIC owner PRIVATE OpenSSL::Crypto)
target_compile_options(run PRIVATE -fno-fast-math -ffp-contract=off)
'''
        self.assertEqual(library_source(source, "run", "case/run/Run.cmake"), (
            ["case/run/Run.cpp", "case/Shared.cpp"], ["owner", "OpenSSL::Crypto"],
            ["-fno-fast-math", "-ffp-contract=off"]))

    def test_reject_unresolved_source_variable(self):
        with self.assertRaisesRegex(ValueError, "Unresolved"):
            library_source('add_library(run STATIC "${OTHER_ROOT}/Run.cpp")', "run", "Run.cmake")

    def test_plain_sources_are_relative_to_the_owning_directory(self):
        self.assertEqual(library_source('add_library(run STATIC Run.cpp ../Shared.cpp)',
                                       "run", "case/run/CMakeLists.txt")[0],
                         ["case/run/Run.cpp", "case/Shared.cpp"])

    def test_reject_duplicate_target_definitions(self):
        with self.assertRaisesRegex(ValueError, "exactly one"):
            library_source('add_library(run STATIC Run.cpp)\nadd_library(run STATIC Old.cpp)', "run", "Run.cmake")

    def test_reject_source_escape(self):
        with self.assertRaisesRegex(ValueError, "escapes"):
            library_source('add_library(run STATIC "../../Outside.cpp")', "run", "Run.cmake")

    def test_reject_nonstatic_profile(self):
        with self.assertRaisesRegex(ValueError, "STATIC"):
            library_source('add_library(run SHARED Run.cpp)', "run", "Run.cmake")


if __name__ == "__main__":
    unittest.main()
