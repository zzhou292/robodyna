#!/usr/bin/env python3
"""Compile a source part's typed declarations; never admit a simulation."""
import argparse
from pathlib import Path
import sys
import zipfile

# Stable package import when this standalone CLI is launched from any cwd.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from modelio.yaris_part import (compile_archive_part, compile_archive_readiness,
                               compile_archive_attachment_readiness, write_report)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-archive', type=Path, required=True)
    parser.add_argument('--part-id', type=int, default=2000157)
    parser.add_argument('--output', type=Path, required=True, help='new JSON file, existing parent directory')
    parser.add_argument('--canonical-assets', type=Path, help='opt-in E2a original-frame canonical geometry')
    parser.add_argument('--quadrature', type=Path, help='qualified 4/8/16 rule JSON, required with canonical assets')
    parser.add_argument('--typed-attachments', action='store_true',
                        help='explicit E2b v2 inventory; requires canonical assets and quadrature, never admits mechanics')
    args = parser.parse_args()
    try:
        if args.output.exists() or args.output.is_symlink() or not args.output.parent.is_dir():
            raise ValueError('output must be a new file with an existing parent directory')
        if (args.canonical_assets is None) != (args.quadrature is None):
            raise ValueError('canonical assets and qualified quadrature must be supplied together')
        if args.typed_attachments and args.canonical_assets is None:
            raise ValueError('typed attachments require canonical assets and qualified quadrature')
        report = (compile_archive_part(args.source_archive, args.part_id) if args.canonical_assets is None else
                  (compile_archive_attachment_readiness if args.typed_attachments else compile_archive_readiness)
                  (args.source_archive, args.canonical_assets, args.quadrature, args.part_id))
        write_report(args.output, report)
    except (OSError, ValueError, UnicodeError, zipfile.BadZipFile) as error:
        print(f'robo-dyna source declarations: {error}', file=sys.stderr)
        return 1
    print(f'Wrote typed source declarations to {args.output}; simulation_ready=false')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
