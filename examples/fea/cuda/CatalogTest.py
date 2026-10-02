"""Check that canonical compile aggregates retain every mapped real executable."""

import ast
import json
from pathlib import Path
import re
import sys
import unittest


def declarations(path):
    result = {}
    for node in ast.parse(path.read_text()).body:
        if not isinstance(node, ast.Expr) or not isinstance(node.value, ast.Call):
            continue
        call = node.value
        if getattr(call.func, 'id', '') not in ('alias', 'filegroup'):
            continue
        attrs = {keyword.arg: ast.literal_eval(keyword.value) for keyword in call.keywords}
        result[attrs['name']] = (call.func.id, attrs)
    return result


class CatalogTest(unittest.TestCase):
    def test_each_entry_names_one_real_unchanged_owner(self):
        self.assertEqual(DOCUMENT['schema'], 'robodyna.retained_cuda_demo_catalog.v1')
        programs = DOCUMENT['programs']
        self.assertEqual(len(programs), 26)
        self.assertEqual(len({entry['source'] for entry in programs}), 26)
        self.assertEqual(len({entry['existing_target'] for entry in programs}), 26)
        for entry in programs:
            self.assertEqual(entry['existing_kind'], 'cc_binary')
            self.assertTrue(entry['source'].startswith('src/fea/legacy/lib_bin/'))
            self.assertRegex(entry['source_sha256'], r'^[0-9a-f]{64}$')
            label = entry['target']
            group, name = label.split('/')[-1].split(':')
            self.assertRegex(name, r'^[a-z][a-z0-9_]*$')
            kind, actual = GROUPS[group][name]
            self.assertEqual(kind, 'alias')
            self.assertEqual(actual['actual'], entry['existing_target'])
            self.assertEqual(set(actual), {'name', 'actual', 'tags'})
            self.assertFalse(entry['runtime_qualified'])

    def test_all_groups_and_optional_backends_are_included(self):
        self.assertEqual(ROOT['all_demos'][1]['srcs'], list(DOCUMENT['groups'].values()))
        for group, rules in GROUPS.items():
            entries = [entry for entry in DOCUMENT['programs'] if entry['group'] == group]
            aliases = {name for name, (kind, _) in rules.items() if kind == 'alias'}
            expected = {entry['target'].split(':')[-1] for entry in entries}
            self.assertEqual(aliases, expected)
            actual = rules['all_demos'][1]['srcs']
            self.assertEqual(set(actual), {':' + name for name in expected})
            self.assertEqual(len(actual), len(expected))
        self.assertEqual(sum(entry['requires_deme'] for entry in DOCUMENT['programs']), 7)
        self.assertTrue(all(entry['requires_cudss_in_existing_closure'] for entry in DOCUMENT['programs']))


if __name__ == '__main__':
    paths = [Path(value) for value in sys.argv[1:]]
    if len(paths) != 7:
        raise RuntimeError('Expected catalog, root BUILD and five group BUILD files')
    DOCUMENT = json.loads(paths[0].read_text())
    ROOT = declarations(paths[1])
    GROUPS = {path.parent.name: declarations(path) for path in paths[2:]}
    sys.argv[1:] = []
    unittest.main()
