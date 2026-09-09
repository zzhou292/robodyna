#!/usr/bin/env python3
"""Compile a source part's typed declarations; never admit a simulation."""
import argparse
from pathlib import Path
import sys
import zipfile

# Stable package import when this standalone CLI is launched from any cwd.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from modelio.yaris_part import compile_archive_part, write_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-archive', type=Path, required=True)
    parser.add_argument('--part-id', type=int, default=2000157)
    parser.add_argument('--output', type=Path, required=True, help='new JSON file, existing parent directory')
    args = parser.parse_args()
    try:
        if args.output.exists() or args.output.is_symlink() or not args.output.parent.is_dir():
            raise ValueError('output must be a new file with an existing parent directory')
        report = compile_archive_part(args.source_archive, args.part_id)
        write_report(args.output, report)
    except (OSError, ValueError, UnicodeError, zipfile.BadZipFile) as error:
        print(f'robo-dyna source declarations: {error}', file=sys.stderr)
        return 1
    print(f'Wrote typed source declarations to {args.output}; simulation_ready=false')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
