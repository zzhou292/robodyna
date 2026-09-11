#!/usr/bin/env python3
"""Resolve original ordinary MAT024 failure declarations beside VehicleSourcePlan V1."""
import argparse
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from modelio.vehicle_section_resolution_io import (compile_archive_vehicle_section_resolution,
                                                   write_vehicle_section_resolution)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source-archive', 'canonical-assets', 'scope-report', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.output.exists():
            raise ValueError('output must be a new path')
        report = compile_archive_vehicle_section_resolution(args.source_archive, args.canonical_assets,
                                                            args.scope_report)
        write_vehicle_section_resolution(args.output, report)
    except (OSError, ValueError) as error:
        print('robo-dyna vehicle section resolution: ' + str(error), file=sys.stderr)
        return 1
    print(report['counts'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
