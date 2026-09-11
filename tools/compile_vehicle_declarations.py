#!/usr/bin/env python3
"""Write authenticated vehicle declaration values without duplicating geometry."""
import argparse
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from modelio.vehicle_declaration_io import compile_archive_vehicle_declarations, write_vehicle_declarations

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source-archive', 'canonical-assets', 'scope-report', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.output.exists(): raise ValueError('output must be a new path')
        result = compile_archive_vehicle_declarations(args.source_archive, args.canonical_assets, args.scope_report)
        write_vehicle_declarations(args.output, result)
    except (OSError, ValueError) as error:
        print('robo-dyna vehicle declarations: ' + str(error), file=sys.stderr); return 1
    print(result['counts']); return 0

if __name__ == '__main__': raise SystemExit(main())
