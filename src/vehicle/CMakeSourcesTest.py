"""Small failure cases for the deliberately limited source-list reader."""

from pathlib import Path
import tempfile
import unittest

from src.vehicle.CMakeSources import library_groups, source_groups


class CMakeSourcesTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "CMakeLists.txt"

    def test_final_library_reuses_nested_group_without_duplicate_owners(self):
        self.path.write_text('''set(CVM_SMALL_FILES one.cpp one.h)
set(CVM_FINAL_FILES ${CVM_SMALL_FILES} two.cpp)
add_library(ChronoModels_vehicle ${CVM_FINAL_FILES} ${CVM_FINAL_FILES})
''')
        groups = source_groups(self.path, "src/models", "CVM")
        final = library_groups(self.path, "ChronoModels_vehicle", groups)
        self.assertEqual(list(final), ["CVM_FINAL_FILES"])
        self.assertEqual(final["CVM_FINAL_FILES"], ["src/models/one.cpp", "src/models/one.h", "src/models/two.cpp"])

    def test_disabled_optional_source_is_not_admitted(self):
        self.path.write_text('''set(CV_TERRAIN_FILES Flat.cpp)
if(CH_USE_OPENCRG)
 set(CV_TERRAIN_FILES ${CV_TERRAIN_FILES} CRG.cpp)
endif()
if(CH_ENABLE_MODULE_FEA)
 list(APPEND CV_TERRAIN_FILES FEA.cpp)
endif()
''')
        result = source_groups(self.path, "src/vehicle", "CV")
        self.assertEqual(result["CV_TERRAIN_FILES"], ["src/vehicle/Flat.cpp", "src/vehicle/FEA.cpp"])

    def test_unknown_condition_or_unresolved_group_rejects(self):
        for text in ('if(UNREVIEWED_FEATURE)\nset(CV_NEW_FILES new.cpp)\nendif()\n',
                     'set(CV_NEW_FILES ${MISSING_GROUP})\n'):
            self.path.write_text(text)
            with self.subTest(text=text), self.assertRaises(ValueError):
                source_groups(self.path, "src/vehicle", "CV")


if __name__ == "__main__":
    unittest.main()
