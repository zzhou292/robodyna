import json
from pathlib import Path
import tempfile
import unittest

from tools.verification.catalog import SCHEMA, human_rows, read_inventory, select
from tools.verification.cmake_evidence import describe, source_mentions
from tools.verification.chrono_inventory import cmake_exposure


def entry(name, kind='demo', module='core', target=None, evidence=None):
    return dict(source=name, kind=kind, module=module, language='cpp',
                bazel={'status':'runnable_target' if target else 'source_only',
                       'targets':[target] if target else []},
                runtime={'evidence':evidence or []},
                cmake={'status':'literal_source_declaration','source_mentions':[],
                       'gate_conditions':['BUILD_DEMOS']})


class CatalogTest(unittest.TestCase):
    def setUp(self):
        self.document = {'schema':SCHEMA,'files':[
            entry('src/demos/core/demo_A.cpp'),
            entry('src/demos/fea/demo_B.cpp',module='fea',target='//examples:fea'),
            entry('src/tests/unit_tests/core/utest_A.cpp',kind='unit_test',
                  target='//tests:original',evidence=[{'scope':'historical run'}])
        ]}

    def test_retention_does_not_imply_exposure_or_qualification(self):
        self.assertEqual(len(select(self.document,status='pending')),1)
        self.assertEqual(len(select(self.document,status='runnable')),2)
        self.assertEqual(len(select(self.document,status='qualified')),1)
        self.assertEqual(select(self.document,kind='demo',status='qualified'),[])

    def test_filters_compose_and_output_keeps_exact_source(self):
        rows=select(self.document,kind='demo',module='fea',language='cpp',pattern='*demo_B.cpp')
        self.assertEqual(len(rows),1)
        self.assertIn('src/demos/fea/demo_B.cpp','\n'.join(human_rows(rows)))
        self.assertIn('//examples:fea','\n'.join(human_rows(rows)))
        pending='\n'.join(human_rows(select(self.document,status='pending')))
        self.assertIn('pending; retained as source only',pending)
        self.assertIn('none recorded',pending)

    def test_historical_source_mismatch_is_visible_in_human_output(self):
        row=entry('src/demos/demo_changed.cpp',evidence=[{'matches_current_source':False}])
        output='\n'.join(human_rows([row]))
        self.assertIn('historical; current source differs',output)
        self.assertNotIn('matching source bytes',output)

    def test_reader_rejects_fake_runnable_and_duplicate_sources(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'inventory.json'
            path.write_text(json.dumps(self.document));read_inventory(path)
            self.document['files'][0]['bazel']['status']='runnable_target'
            path.write_text(json.dumps(self.document))
            with self.assertRaisesRegex(ValueError,'actual declared target'):read_inventory(path)
            self.document['files'][0]['bazel']['status']='source_only'
            self.document['files'].append(self.document['files'][0])
            path.write_text(json.dumps(self.document))
            with self.assertRaisesRegex(ValueError,'duplicate'):read_inventory(path)

    def test_comments_are_not_cmake_source_admission(self):
        parsed=describe('CMakeLists.txt', '''# set(TESTS utest_disabled)
if(CH_ENABLE_MODULE_FEA)
  set(TESTS utest_real) # utest_disabled
  build_utests(NO "${TESTS}" "fea" Chrono_core)
endif()
message(STATUS "C# is not a comment")
''')
        self.assertEqual(source_mentions(parsed,'utest_disabled.cpp'),[])
        self.assertEqual(source_mentions(parsed,'utest_real.cpp')[0]['conditions'],['CH_ENABLE_MODULE_FEA'])
        exposure=cmake_exposure('src/tests/unit_tests/fea/utest_real.cpp','unit_test',
                               {'src/tests/unit_tests/fea/CMakeLists.txt':parsed},{})
        self.assertEqual(exposure['ctest'],'explicitly_not_registered')
        self.assertIn('BUILD_TESTING',exposure['gate_conditions'])

    def test_escaped_unquoted_shell_quotes_do_not_hide_later_cmake(self):
        parsed=describe('CMakeLists.txt', r'''add_custom_command(COMMAND for f in \"${dir}\"/*.cs; do ( echo test ) done)
message(STATUS "Add C# CORE")
set(TESTS utest_after)
''')
        self.assertEqual(len(source_mentions(parsed,'utest_after.cpp')),1)


if __name__=='__main__':unittest.main()
