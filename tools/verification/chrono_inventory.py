"""Audit pinned Chrono demo/test sources and static exposure without configuring builds."""

import argparse
import ast
import collections
import datetime
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import subprocess

from tools.verification.catalog import SCHEMA
from tools.verification.cmake_evidence import describe, source_mentions

PREFIXES = ('src/demos/', 'src/tests/unit_tests/')
LANGUAGES = {'.cpp': 'cpp', '.py': 'python', '.cs': 'csharp'}


def git(repo, *args):
    return subprocess.check_output(['git', *args], cwd=repo)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def tree_files(repo, tree):
    result = {}
    for record in git(repo, 'ls-tree', '-rz', '--full-tree', tree).split(b'\0'):
        if record:
            meta, path = record.split(b'\t', 1)
            mode, kind, oid = meta.decode().split()
            if kind == 'blob':
                result[path.decode()] = dict(mode=mode, git_blob=oid)
    return result


def literal_bazel_targets(repo, prefix):
    """Inspect real cc_binary/cc_test srcs, excluding exports and data-only groups."""
    result = collections.defaultdict(list)
    names = git(repo, 'ls-files', '-c', '-o', '--exclude-standard', '-z').decode().split('\0')
    for name in sorted(set(names)):
        if (not name.endswith('BUILD.bazel') or name.startswith('src/fea/legacy/')
                or not (repo / name).is_file()):
            continue
        for node in ast.parse((repo / name).read_text()).body:
            if not isinstance(node, ast.Expr) or not isinstance(node.value, ast.Call):
                continue
            call = node.value
            if not isinstance(call.func, ast.Name) or call.func.id not in ('cc_binary', 'cc_test'):
                continue
            attrs = {kw.arg: kw.value for kw in call.keywords}
            if 'name' not in attrs or 'srcs' not in attrs:
                continue
            target = '//' + str(PurePosixPath(name).parent) + ':' + ast.literal_eval(attrs['name'])
            for value in ast.walk(attrs['srcs']):
                if isinstance(value, ast.Constant) and isinstance(value.value, str):
                    marker = '//' + prefix + ':'
                    if value.value.startswith(marker):
                        result[value.value[len(marker):]].append(dict(target=target, rule=call.func.id,
                                                                    declaration=name, line=node.lineno))
    return result


def file_kind(name):
    p = PurePosixPath(name)
    if p.suffix not in LANGUAGES:
        return 'support'
    if p.name.startswith(('utest_', 'pyutest_')):
        return 'unit_test'
    return 'demo' if p.name.startswith('demo_') else 'support'


def module_name(name):
    p = PurePosixPath(name)
    if name.startswith('src/demos/'):
        parts = p.parts[2:-1]
    elif name.startswith('src/tests/unit_tests/'):
        parts = p.parts[3:-1]
    else:
        return 'fmi_template'
    if parts and parts[0] in ('python', 'csharp'):
        parts = parts[1:]
    return parts[0] if parts else 'shared'


def cmake_index(repo, prefix, files):
    result, parents = {}, collections.defaultdict(list)
    for name in files:
        if PurePosixPath(name).name != 'CMakeLists.txt' or 'chrono_thirdparty/' in name:
            continue
        if not (repo / prefix / name).is_file():
            continue  # The retention rows report a missing build file explicitly.
        description = describe(prefix + '/' + name, (repo / prefix / name).read_text())
        result[name] = description
        for row in description['subdirectories']:
            argument = row['arguments'].split()[0].strip('"')
            if '$' not in argument:
                child = str(PurePosixPath(name).parent / argument / 'CMakeLists.txt')
                parents[child].append(dict(parent=name, line=row['line'], conditions=row['conditions']))
    return result, parents


def cmake_exposure(name, kind, cmakes, parents):
    owner = str(PurePosixPath(name).parent / 'CMakeLists.txt')
    matches = source_mentions(cmakes[owner], PurePosixPath(name).name) if owner in cmakes else []
    gates = (['BUILD_TESTING'] if name.startswith('src/tests/unit_tests/') else
             ['BUILD_DEMOS'] if name.startswith('src/demos/') else [])
    path, visited, route = owner, set(), []
    while path not in visited and parents.get(path):
        visited.add(path)
        edge = parents[path][0]
        route.append(edge)
        gates.extend(edge['conditions'])
        path = edge['parent']
    route_kind = 'conditional_root_route' if path == 'CMakeLists.txt' else 'no_literal_root_route'
    if name.startswith('template_'):
        route_kind = 'standalone_template_project'
    if name.startswith('src/demos/csharp/') and path == 'src/demos/csharp/CMakeLists.txt':
        route_kind = 'conditional_csharp_subbuild'
        for row in cmakes.get('src/demos/CMakeLists.txt', {}).get('commands', []):
            if row['command'] == 'add_custom_target' and row['arguments'].startswith('Csharp_demos '):
                gates.extend(row['conditions'])
        gates.append('CMAKE_CSharp_COMPILER must be available to the retained C# subbuild')
    for row in matches:
        gates.extend(row['conditions'])
    sources = [dict(path=cmakes[owner]['path'], line=row['line'], command=row['command']) for row in matches]
    status = 'literal_source_declaration' if matches else 'no_literal_source_declaration'
    ctest = 'not_applicable'
    if kind == 'unit_test':
        ctest = 'conditional_registration'
        calls = cmakes.get(owner, {}).get('build_calls', [])
        if any(row['command'] in ('build_utests', 'build_pyutests') and
               row['arguments'].split()[0].upper() == 'NO' for row in calls):
            ctest = 'explicitly_not_registered'
        if not matches:
            ctest = 'no_literal_source_declaration'
    if name.startswith('src/demos/python/'):
        status = 'python_directory_installed_not_executable_target'
        route_kind = 'python_directory_install'
        install = cmakes.get('src/chrono_swig/chrono_python/CMakeLists.txt', {})
        sources = [dict(path=install['path'], line=row['line'], command='install(DIRECTORY)')
                   for row in install.get('commands', [])
                   if row['command'] == 'install' and '/src/demos/python/' in row['arguments']]
        gates = ['CH_ENABLE_MODULE_PYTHON; installed directory; module SDKs depend on script imports']
    return dict(status=status, owner=cmakes.get(owner, {}).get('path'), source_mentions=sources,
                gate_conditions=list(dict.fromkeys(gates)), ancestor_routes=route,
                route=route_kind,
                ctest=ctest, qualification='static source evidence; no CMake configure/build performed')


def audit(repo):
    repo = Path(repo).resolve()
    manifest = json.loads((repo / 'docs/migration/SOURCES.json').read_text())
    pin = next(row for row in manifest['sources'] if row['component'] == 'chrono_capabilities')
    original = tree_files(repo, pin['source_tree'])
    selected = {name: row for name, row in original.items() if name.startswith(PREFIXES)}
    supplemental = [name for name in original if file_kind(name) in ('demo', 'unit_test') and name not in selected]
    selected.update({name: original[name] for name in supplemental})
    cmakes, parents = cmake_index(repo, pin['path'], original)
    targets = literal_bazel_targets(repo, pin['path'])
    examples = json.loads((repo / 'examples/QUALIFICATION.json').read_text())
    delivery = {row['source_demo']['path']: (index, row) for index, row in enumerate(examples['examples'])}
    files = []
    for name, source in sorted(selected.items()):
        full = pin['path'] + '/' + name
        path = repo / full
        data = path.read_bytes() if path.is_file() else None
        blob = hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest() if data is not None else None
        kind = file_kind(name)
        evidence = []
        if full in delivery:
            index, entry = delivery[full]
            evidence.append(dict(record='examples/QUALIFICATION.json', example_index=index,
                                 scope='named six-second CPU dynamics and Vulkan capture/video qualification',
                                 source_sha256=entry['source_demo']['sha256'],
                                 matches_current_source=data is not None and digest(data) == entry['source_demo']['sha256']))
        if name == 'src/tests/unit_tests/physics/utest_CH_composite_inertia.cpp':
            relative = 'crash-work/reports/robodyna-participant-services-native-1'
            receipt, log = repo.parent / (relative + '.json'), repo.parent / (relative + '.log')
            if receipt.is_file() and log.is_file():
                record = json.loads(receipt.read_text())
                if (record.get('status') == 'passed' and record.get('exit_code') == 0 and
                        record.get('process_scope', {}).get('cleanup') == 'complete' and
                        re.search(r'^//src/mechanics/inertia:inertia_test\s+PASSED', log.read_text(), re.MULTILINE)):
                    evidence.append(dict(record='outer:' + relative + '.json', receipt_sha256=digest(receipt.read_bytes()),
                                         log='outer:' + relative + '.log', log_sha256=digest(log.read_bytes()),
                                         target='//src/mechanics/inertia:inertia_test',
                                         scope='original test source included in this passing target; not the whole inherited test suite'))
        files.append(dict(source=full, original_path=name, kind=kind, module=module_name(name),
                          language=LANGUAGES.get(PurePosixPath(name).suffix),
                          original_git_blob=source['git_blob'], current_git_blob=blob,
                          current_sha256=digest(data) if data is not None else None,
                          bytes=len(data) if data is not None else None,
                          retention='missing' if data is None else 'unchanged' if blob == source['git_blob'] else 'modified',
                          scope='standard_tree' if name.startswith(PREFIXES) else 'supplemental_named_example',
                          cmake=cmake_exposure(name, kind, cmakes, parents),
                          bazel={'status':'runnable_target' if targets[name] else 'source_only',
                                 'targets':[row['target'] for row in targets[name]], 'declarations':targets[name]},
                          runtime={'evidence':evidence, 'all_inherited_cases_qualified':False}))
    dependencies = []
    for row in json.loads((repo / 'docs/migration/DEPENDENCIES.json').read_text())['dependencies']:
        if row['name'].startswith('chrono-'):
            path = repo / row['path']
            dependencies.append(dict(name=row['name'], path=row['path'], recorded_commit=row['recorded_commit'],
                                     directory_present=path.is_dir(), has_cmake_project=(path/'CMakeLists.txt').is_file(),
                                     entries=len(list(path.iterdir())) if path.is_dir() else 0))
    # Store the source evidence once rather than repeating whole CMake files per example.
    descriptions = {name:{k:v for k,v in row.items() if k!='commands'} for name,row in cmakes.items()}
    groups = {}
    for scope in ('src/demos/', 'src/tests/unit_tests/'):
        rows = [row for row in files if row['original_path'].startswith(scope)]
        groups[scope] = dict(files=len(rows), retention=dict(collections.Counter(r['retention'] for r in rows)),
                            languages=dict(collections.Counter(r['language'] for r in rows if r['language'])),
                            program_sources=sum(r['kind'] != 'support' for r in rows))
    return dict(schema=SCHEMA, generated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                audited_head=git(repo,'rev-parse','HEAD').decode().strip(), source_pin=pin,
                methodology={'retention':'Every pinned blob in standard demo/unit-test trees plus named demo/utest sources elsewhere; Git blob identity checked against current bytes.',
                             'cmake':'Lexical uncommented command references and ancestry; conditions are recorded, not evaluated. No SDK/configure success implied.',
                             'bazel':'Only literal imported srcs of actual cc_binary/cc_test declarations; source exports/filegroups are not executable exposure.',
                             'runtime':'Only named closed evidence; historical demo source hashes may differ after maintained branding.'},
                summary=groups, supplemental_named_sources=supplemental,
                missing=[r['source'] for r in files if r['retention']=='missing'],
                inherited_gitlinks=dependencies, cmake_files=descriptions, files=files)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    document = audit(args.repo)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n')
    print(json.dumps({'summary':document['summary'], 'missing':document['missing']}, indent=2))


if __name__ == '__main__':
    main()
