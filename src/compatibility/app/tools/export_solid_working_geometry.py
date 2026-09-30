#!/usr/bin/env python3
"""Export authenticated original radiator geometry; root owns full-source runs."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from modelio.solid_geometry_export import export_geometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', type=Path, required=True)
    parser.add_argument('--source-member', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = export_geometry(args.assets, args.source_member, args.output, [2000063],
        'c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8',
        expected_parents=1345, expected_nodes=2904)
    print(f"Exported {report['solid_count']} original solids / {report['node_count']} nodes")


if __name__ == '__main__':
    main()
