#!/usr/bin/env python3
"""Create-only full original Yaris shell scope; never admits a solver."""
import argparse
from pathlib import Path
import sys
import zipfile
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from modelio.yaris_full_shell import compile_full_shell_scope,write_full_shell_scope


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-archive',type=Path,required=True)
    parser.add_argument('--canonical-assets',type=Path,required=True)
    parser.add_argument('--tire-policy',choices=('retain_all','omit_original_tire_shells'),default='retain_all')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    try:
        if args.output.exists():raise ValueError('output must be a new path')
        report=compile_full_shell_scope(args.source_archive,args.canonical_assets,args.tire_policy)
        write_full_shell_scope(args.output,report)
    except (OSError,ValueError,UnicodeError,zipfile.BadZipFile) as error:
        print('robo-dyna full-shell scope: '+str(error),file=sys.stderr);return 1
    print(f'Wrote {report["coverage"]["retained"]["shells"]} retained shells; simulation_ready=false')
    return 0


if __name__=='__main__':raise SystemExit(main())
