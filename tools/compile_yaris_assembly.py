#!/usr/bin/env python3
"""Inventory selected whole source parts and every known outgoing interface."""
import argparse
from pathlib import Path
import sys
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from modelio.yaris_assembly import compile_archive_assembly, write_assembly_report, YARIS_CONNECTOR_PARTS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-archive', required=True, type=Path)
    parser.add_argument('--canonical-assets', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path, help='new file in an existing directory')
    parser.add_argument('--boundary-policy', choices=('unassigned', 'released_external_connections'), default='unassigned')
    parser.add_argument('--part-ids', nargs='+', type=int, default=YARIS_CONNECTOR_PARTS,
                        help='complete source part IDs; defaults to the original six-part component')
    args = parser.parse_args()
    try:
        if args.output.exists() or args.output.is_symlink() or not args.output.parent.is_dir():
            raise ValueError('output must be a new file with an existing parent directory')
        report = compile_archive_assembly(args.source_archive, args.canonical_assets,
                                         part_ids=args.part_ids,
                                         boundary_policy=args.boundary_policy)
        write_assembly_report(args.output, report)
    except (OSError, ValueError, UnicodeError, zipfile.BadZipFile) as error:
        print(f'robo-dyna assembly inventory: {error}', file=sys.stderr)
        return 1
    print(f'Wrote {report["counts"]["parts"]} whole parts to {args.output}; simulation_ready=false')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
