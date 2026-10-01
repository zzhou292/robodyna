"""List retained Chrono examples/tests, their source gates and real root targets."""

import argparse
import json
from pathlib import Path

from tools.verification.catalog import human_rows, read_inventory, select


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inventory', default=None)
    parser.add_argument('--kind', choices=['demo', 'unit_test'])
    parser.add_argument('--module')
    parser.add_argument('--language', choices=['cpp', 'python', 'csharp'])
    parser.add_argument('--status', choices=['runnable', 'pending', 'qualified'],
                        help='qualified selects recorded historical evidence, not current dependency qualification')
    parser.add_argument('--path', default='*', help='Case-sensitive source-path glob')
    parser.add_argument('--format', choices=['text', 'json'], default='text')
    args = parser.parse_args()
    path = Path(args.inventory) if args.inventory else Path(__file__).resolve().parents[2] / 'docs/verification/CHRONO_DEMOS_TESTS_INVENTORY.json'
    if not path.is_file() and args.inventory:
        try:
            from python.runfiles import runfiles
        except ImportError:
            parser.error('Inventory file does not exist: ' + args.inventory)
        resolver = runfiles.Create()
        resolved = resolver.Rlocation(args.inventory) if resolver else None
        if resolved:
            path = Path(resolved)
    if not path.is_file():
        parser.error('Inventory file does not exist: ' + str(path))
    document = read_inventory(path)
    rows = select(document, args.kind, args.module, args.language, args.status, args.path)
    if args.format == 'json':
        print(json.dumps({'source_pin': document['source_pin'], 'count': len(rows), 'files': rows}, indent=2))
    else:
        print(f"{len(rows)} matching retained sources. Discovery is not build/runtime qualification.")
        for line in human_rows(rows):
            print(line)


if __name__ == '__main__':
    main()
