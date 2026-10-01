"""Create bounded immutable parser snapshots from Git or the current workspace."""

import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

from tools.bindings import declaration_registry


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def selected_name(name):
    return (name.startswith(("src/compatibility/chrono/src/", "include/robodyna/"))
            and Path(name).suffix in (".h", ".hpp", ".hxx", ".inl", ".inc", ".i"))


def historical(repo, output, reference):
    commit = subprocess.check_output(["git", "rev-parse", "--verify", reference + "^{commit}"],
                                     cwd=repo, text=True).strip()
    inputs, total = {}, 0
    command = ["git", "archive", "--format=tar", commit, "src/compatibility/chrono/src"]
    process = subprocess.Popen(command, cwd=repo, stdout=subprocess.PIPE)
    try:
        with tarfile.open(fileobj=process.stdout, mode="r|") as archive:
            for member in archive:
                if not member.isfile() or not selected_name(member.name):
                    continue
                name = Path(member.name)
                if name.is_absolute() or ".." in name.parts:
                    raise ValueError("Invalid historical source path")
                total += member.size
                if total > 256 * 1024 * 1024:
                    raise ValueError("Historical parser inputs exceed 256 MiB")
                data = archive.extractfile(member).read()
                target = output / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(data)
                inputs[member.name] = hashlib.sha256(data).hexdigest()
        if process.wait():
            raise ValueError("Historical Git archive failed")
    finally:
        process.stdout.close()
        if process.poll() is None:
            process.terminate()
            process.wait()
    return {"source_commit": commit, "source_inputs": inputs, "source_bytes": total}


def snapshot(repo, output, reference=None, fea_enabled=True):
    """Copy parser inputs once and authenticate generated views; never compile."""
    output.mkdir()
    if reference is not None:
        record = historical(repo, output, reference)
        record.update(schema="robodyna.swig_parser_inputs.v1", candidate=False,
                      scope="Genuine historical Git parser inputs; not a compiled backend")
    else:
        from tools.bindings.declaration_view import generate
        head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
        names = subprocess.check_output(["git", "ls-files", "-c", "-o", "--exclude-standard", "-z"], cwd=repo).decode().split("\0")
        inputs, total = {}, 0
        for name in sorted({name for name in names if selected_name(name)}):
            data = (repo / name).read_bytes()
            total += len(data)
            if total > 256 * 1024 * 1024:
                raise ValueError("Parser inputs exceed 256 MiB")
            target = output / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
            inputs[name] = hashlib.sha256(data).hexdigest()
        generated, support = {}, {}
        registry_bytes = (repo / declaration_registry.REGISTRY).read_bytes()
        registry_target = output / declaration_registry.REGISTRY
        registry_target.parent.mkdir(parents=True, exist_ok=True)
        registry_target.write_bytes(registry_bytes)
        support[declaration_registry.REGISTRY] = hashlib.sha256(registry_bytes).hexdigest()
        for entry in declaration_registry.enabled(repo, fea_enabled):
            contract_name, ledger_name = entry["contract"], entry["ledger"]
            for name in (contract_name, ledger_name):
                data = (repo / name).read_bytes()
                target = output / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(data)
                support[name] = hashlib.sha256(data).hexdigest()
            contract = json.loads((output / contract_name).read_text())
            view = output / "swig_generated" / contract["output"]
            view.parent.mkdir(parents=True, exist_ok=True)
            generated[contract["output"]] = generate(output, output / ledger_name, contract["original_path"],
                                                     contract["expected_original_sha256"], contract["expected_ledger_sha256"], view)
        for name, expected in (inputs | support).items():
            if digest(repo / name) != expected:
                raise ValueError("Live source changed during snapshot: " + name)
        record = {"schema": "robodyna.swig_parser_inputs.v1", "candidate": True,
                  "source_commit": head, "source_inputs": inputs, "support_inputs": support,
                  "source_bytes": total, "generated_declaration_views": generated,
                  "scope": "Frozen current source bytes with authenticated parser-only views; not compilation/runtime"}
    (output / "parser-inputs.json").write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
    return record
