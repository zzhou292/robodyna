#!/usr/bin/env python3
"""Import one pinned Git tree, retaining its history and preserving working copies."""

import argparse
import fcntl
import json
import os
from pathlib import Path, PurePosixPath
import subprocess


def git(repository, *arguments, capture=True):
    environment = dict(os.environ, GIT_LFS_SKIP_SMUDGE="1", GIT_TERMINAL_PROMPT="0")
    result = subprocess.run(
        ["git", "-C", str(repository), "-c", "gc.auto=0",
         "-c", "maintenance.auto=false", *arguments],
        text=True, capture_output=capture, env=environment, check=True,
    )
    return result.stdout.strip() if capture else None


def validate_prefix(prefix):
    path = PurePosixPath(prefix)
    if path.is_absolute() or not path.parts or ".." in path.parts:
        raise ValueError("Import prefix must be a nonempty relative path without '..'")
    if str(path) != prefix or any(part == ".git" for part in path.parts):
        raise ValueError("Import prefix must be canonical and cannot contain .git")
    return prefix


def import_tree(repository, source_ref, expected_commit, prefix, author_lock):
    repository = repository.resolve(strict=True)
    prefix = validate_prefix(prefix)
    with author_lock.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        source_commit = git(repository, "rev-parse", source_ref + "^{commit}")
        if source_commit != expected_commit:
            raise ValueError("Source reference does not match the pinned commit")
        original_head = git(repository, "rev-parse", "HEAD")
        git(repository, "diff", "--quiet")
        git(repository, "diff", "--cached", "--quiet")
        if git(repository, "ls-tree", "HEAD", "--", prefix):
            raise ValueError("Import destination is already tracked")
        merge_head = Path(git(repository, "rev-parse", "--git-path", "MERGE_HEAD"))
        if not merge_head.is_absolute():
            merge_head = repository / merge_head
        if merge_head.exists():
            raise ValueError("An existing merge must be resolved first")
        source_tree = git(repository, "rev-parse", source_commit + "^{tree}")
        # The ours merge retains history; read-tree then installs the exact source
        # under its owned prefix. No reset, deletion or source-repo write occurs.
        git(repository, "merge", "--no-commit", "--no-ff", "--allow-unrelated-histories",
            "-s", "ours", source_commit, capture=False)
        git(repository, "read-tree", "--prefix=" + prefix + "/", "-u", source_commit,
            capture=False)
        pending_tree = git(repository, "write-tree")
        if git(repository, "rev-parse", pending_tree + ":" + prefix) != source_tree:
            raise RuntimeError("Staged import tree differs from its source; merge remains for review")
        git(repository, "commit", "-m", "Import preserved source history at " + prefix,
            capture=False)
        imported_head = git(repository, "rev-parse", "HEAD")
        if git(repository, "rev-parse", imported_head + ":" + prefix) != source_tree:
            raise RuntimeError("Committed import tree differs from source")
        git(repository, "merge-base", "--is-ancestor", source_commit, imported_head)
        return {
            "previous_head": original_head,
            "import_commit": imported_head,
            "source_commit": source_commit,
            "source_tree": source_tree,
            "prefix": prefix,
            "exact_tree": True,
            "source_history_reachable": True,
            "lfs_smudge_skipped": True,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository", type=Path, required=True)
    parser.add_argument("--source-ref", required=True)
    parser.add_argument("--expected-commit", required=True)
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--author-lock", type=Path, required=True)
    arguments = parser.parse_args()
    result = import_tree(
        arguments.repository, arguments.source_ref, arguments.expected_commit,
        arguments.prefix, arguments.author_lock,
    )
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
