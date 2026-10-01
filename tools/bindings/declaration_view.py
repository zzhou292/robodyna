"""Generate an authenticated, parser-only compatibility declaration view."""

import argparse
import hashlib
import json
from pathlib import Path

from tools.migration.source_transform import index_entries, original_bytes, relative_file


def digest(data):
    return hashlib.sha256(data).hexdigest()


def generate(root, ledger, original_path, expected_original, expected_ledger, output):
    ledger_bytes = Path(ledger).read_bytes()
    if digest(ledger_bytes) != expected_ledger:
        raise ValueError("Declaration transformation differs from its reviewed ledger pin")
    entry = index_entries(json.loads(ledger_bytes))[original_path]
    if entry["original_sha256"] != expected_original:
        raise ValueError("Declaration view differs from its immutable original identity")
    restored = original_bytes(root, entry)
    header = (b"// Generated parser metadata, authenticated against canonical source.\n"
              b"#ifndef SWIG\n#error This declaration view is for SWIG only; include the real Robodyna API.\n#endif\n"
              + restored)
    output = Path(output)
    with output.open("xb") as stream:
        stream.write(header)
    receipt = {"schema": "robodyna.swig_declaration_view.v1",
               "original_path": original_path, "original_sha256": digest(restored),
               "canonical_path": entry["canonical_path"],
               "canonical_sha256": digest(relative_file(root, entry["canonical_path"]).read_bytes()),
               "ledger_sha256": digest(ledger_bytes), "view_sha256": digest(header),
               "scope": "SWIG parser metadata only; generated C++ must include the actual canonical/alias header"}
    with output.with_suffix(output.suffix + ".json").open("x") as stream:
        json.dump(receipt, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return receipt


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--ledger", type=Path, required=True)
    parser.add_argument("--original-path", required=True)
    parser.add_argument("--expected-original-sha256", required=True)
    parser.add_argument("--expected-ledger-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    generate(args.root, args.ledger, args.original_path, args.expected_original_sha256,
             args.expected_ledger_sha256, args.output)
