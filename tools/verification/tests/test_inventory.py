import json
from pathlib import Path
import subprocess
import tempfile
import unittest

from tools.verification.chrono_inventory import audit, literal_bazel_targets


class InventoryTest(unittest.TestCase):
    def test_only_owned_native_demo_macro_is_admitted(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            subprocess.run(['git', 'init', '-q'], cwd=root, check=True)
            source = '//src/compatibility/chrono:src/demos/core/demo_A.cpp'
            for name, declaration in {
                'owned': 'load("//build_defs/examples:defs.bzl", "robodyna_cpp_demo")\nrobodyna_cpp_demo',
                'aliased': 'load("//build_defs/examples:defs.bzl", demo="robodyna_cpp_demo")\ndemo',
                'unrelated': 'load("//other:defs.bzl", "robodyna_cpp_demo")\nrobodyna_cpp_demo',
                'data_only': 'filegroup',
            }.items():
                folder = root / name
                folder.mkdir()
                (folder / 'BUILD.bazel').write_text(declaration + '(name="sample", srcs=[' + json.dumps(source) + '])\n')
            records = literal_bazel_targets(root, 'src/compatibility/chrono')
            rows = records['src/demos/core/demo_A.cpp']
            self.assertEqual({row['target'] for row in rows}, {'//owned:sample', '//aliased:sample'})
            self.assertEqual({row['rule'] for row in rows}, {'cc_binary'})

    def test_real_git_identity_distinguishes_missing_modified_and_source_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            def git(*args):
                return subprocess.check_output(['git',*args],cwd=root,stderr=subprocess.DEVNULL).decode().strip()
            git('init','-q')
            originals={
                'src/demos/core/CMakeLists.txt':'set(DEMOS demo_A demo_B)\n',
                'src/demos/core/demo_A.cpp':'int main() { return 0; }\n',
                'src/demos/core/demo_B.cpp':'int main() { return 1; }\n',
                'src/tests/unit_tests/core/utest_A.cpp':'// retained test\n',
                'src/tests/unit_tests/core/CMakeLists.txt':'set(TESTS utest_A)\nbuild_utests(YES "${TESTS}" core Chrono_core)\n',
            }
            for name,data in originals.items():
                path=root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(data)
            git('add','src');git('-c','user.name=Fixture','-c','user.email=fixture@example.invalid','commit','-qm','source fixture')
            commit,tree=git('rev-parse','HEAD'),git('rev-parse','HEAD^{tree}')
            prefix='src/compatibility/chrono'
            for name,data in originals.items():
                path=root/prefix/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(data)
            (root/prefix/'src/demos/core/demo_A.cpp').write_text('// maintained hook\n'+originals['src/demos/core/demo_A.cpp'])
            (root/prefix/'src/demos/core/demo_B.cpp').unlink()
            for name,document in {
                'docs/migration/SOURCES.json':{'sources':[{'component':'chrono_capabilities','path':prefix,'source_commit':commit,'source_tree':tree}]},
                'docs/migration/DEPENDENCIES.json':{'dependencies':[]},
                'examples/QUALIFICATION.json':{'examples':[]},
            }.items():
                path=root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(document))
            build=root/'examples/BUILD.bazel'
            build.write_text('cc_binary(name="original_a", srcs=["//src/compatibility/chrono:src/demos/core/demo_A.cpp"])\n')
            document=audit(root);rows={r['original_path']:r for r in document['files']}
            a=rows['src/demos/core/demo_A.cpp'];b=rows['src/demos/core/demo_B.cpp'];test=rows['src/tests/unit_tests/core/utest_A.cpp']
            self.assertEqual(a['retention'],'modified')
            self.assertNotEqual(a['original_git_blob'],a['current_git_blob'])
            self.assertEqual(a['bazel']['targets'],['//examples:original_a'])
            self.assertEqual(b['retention'],'missing')
            self.assertEqual(document['missing'],[prefix+'/src/demos/core/demo_B.cpp'])
            self.assertEqual(test['retention'],'unchanged')
            self.assertEqual(test['bazel']['status'],'source_only')
            self.assertEqual(test['runtime']['evidence'],[])
            self.assertEqual(document['summary']['src/demos/']['program_sources'],2)
            (root/prefix/'src/demos/core/CMakeLists.txt').unlink()
            missing_build=audit(root)
            self.assertIn(prefix+'/src/demos/core/CMakeLists.txt',missing_build['missing'])


if __name__=='__main__':unittest.main()
