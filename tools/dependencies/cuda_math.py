"""Provision pinned CUDA math components without modifying the host toolkit."""

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import posixpath
import resource
import tarfile
import urllib.request


def file_hash(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def admit_archive(path, entry):
    if path.stat().st_size != entry['bytes'] or file_hash(path) != entry['sha256']:
        raise ValueError('Archive differs from its pinned NVIDIA/source identity: ' + str(path))


def download(manifest, directory):
    directory.mkdir(parents=True, exist_ok=True)
    for name, entry in manifest['components'].items():
        path = directory / entry['archive']
        if path.exists():
            admit_archive(path, entry)
            print('verified cached', name, flush=True)
            continue
        partial = path.with_suffix(path.suffix + '.partial')
        if partial.exists():
            raise ValueError('Preserved incomplete download requires inspection: ' + str(partial))
        count, next_report = 0, 64 * 1024 * 1024
        digest = hashlib.sha256()
        with urllib.request.urlopen(entry['url'], timeout=60) as response, partial.open('xb') as output:
            while block := response.read(1024 * 1024):
                count += len(block)
                if count > entry['bytes']:
                    raise ValueError('Response exceeds pinned archive length: ' + name)
                output.write(block)
                digest.update(block)
                if count >= next_report:
                    print(name, count, '/', entry['bytes'], flush=True)
                    next_report += 64 * 1024 * 1024
        if count != entry['bytes'] or digest.hexdigest() != entry['sha256']:
            raise ValueError('Downloaded archive failed exact length/SHA256 admission: ' + name)
        partial.rename(path)
        print('downloaded verified', name, entry['version'], flush=True)


def strip_members(archive):
    result, roots = [], set()
    for member in archive.getmembers():
        path = PurePosixPath(member.name)
        if path.is_absolute() or '..' in path.parts or not path.parts:
            raise ValueError('Unsafe archive path: ' + member.name)
        roots.add(path.parts[0])
        if len(path.parts) == 1:
            if not member.isdir():
                raise ValueError('Archive must have one enclosing directory')
            continue
        if not (member.isfile() or member.isdir() or member.issym() or member.islnk()):
            raise ValueError('Unsupported special archive member: ' + member.name)
        item = copy.copy(member)
        item.name = str(PurePosixPath(*path.parts[1:]))
        item.mode &= 0o777
        if item.issym():
            target = posixpath.normpath(posixpath.join(posixpath.dirname(item.name), item.linkname))
            if (item.linkname.startswith('/') or '..' in PurePosixPath(item.linkname).parts
                    or target == '..' or target.startswith('../')):
                raise ValueError('Archive symlink escapes component: ' + member.name)
        elif item.islnk():
            link = PurePosixPath(item.linkname)
            if link.is_absolute() or '..' in link.parts or link.parts[0] != path.parts[0]:
                raise ValueError('Archive hardlink escapes component: ' + member.name)
            item.linkname = str(PurePosixPath(*link.parts[1:]))
        result.append(item)
    if len(roots) != 1:
        raise ValueError('Expected one component root per archive')
    return result


def extract(manifest, directory, install):
    if install.exists():
        raise ValueError('Choose a new create-only SDK installation directory: ' + str(install))
    install.mkdir(parents=True)
    components = {}
    for name, entry in manifest['components'].items():
        path = directory / entry['archive']
        admit_archive(path, entry)
        target = install / 'components' / name
        target.mkdir(parents=True)
        with tarfile.open(path, 'r:*') as archive:
            archive.extractall(target, members=strip_members(archive))
        files = {}
        for member in sorted(target.rglob('*')):
            relative = member.relative_to(target).as_posix()
            if member.is_symlink():
                files[relative] = {'symlink': os.readlink(member)}
            elif member.is_file():
                files[relative] = {'bytes': member.stat().st_size, 'sha256': file_hash(member)}
        components[name] = dict(entry, root='components/' + name, files=files)
        print('extracted verified', name, len(files), 'files', flush=True)
    receipt = dict(schema='robodyna.cuda_math_sdk.v1', cuda_release=manifest['cuda_release'],
                   nvcc_version=manifest['nvcc_version'], manifests=manifest['manifests'],
                   cccl=manifest['cccl'], components=components,
                   scope='Workspace-local headers/libraries. Host CUDA compiler, runtime and CCCL remain separately owned.')
    (install / 'sdk.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print('SDK ready', install, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--downloads', type=Path, required=True)
    parser.add_argument('--install', type=Path)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    if manifest.get('schema') != 'robodyna.cuda_math_downloads.v1':
        raise ValueError('Unsupported download manifest')
    if args.install:
        extract(manifest, args.downloads, args.install)
    else:
        # Lightweight streaming transfer may share the workstation with a build.
        # Extraction is separate and must use the normal workstation guard/lock.
        affinity = sorted(os.sched_getaffinity(0))
        selected = [cpu for cpu in affinity if cpu < 8] or affinity
        os.sched_setaffinity(0, {selected[-1]})
        resource.setrlimit(resource.RLIMIT_AS, (1024**3, 1024**3))
        download(manifest, args.downloads)


if __name__ == '__main__':
    main()
