"""Create-only source and native reference deck export; does not run a solver."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

from viewer.file_integrity import sha256_file
from .definition import load
from .mesh import build
from .native import starter, engine


def export(source, destination):
    source = Path(source).resolve()
    with source.open('rb') as input_file:
        source_bytes = input_file.read((128 << 10)+1)
    if len(source_bytes) > 128 << 10:
        raise ValueError('Scene declaration exceeds128KiB')
    scene = load(source)
    mesh = build(scene)
    decks = {'contact_scene_0000.rad':starter(scene,mesh), 'contact_scene_0001.rad':engine(scene)}
    # Evaluate/validate every representation before creating the destination.
    record = {'schema':'robo_dyna.native_contact_scene_export.v1',
              'scope':'declared source only; runtime controls and physical trajectory unqualified',
              'source_sha256':hashlib.sha256(source_bytes).hexdigest(), 'scene':asdict(scene), 'mesh':asdict(mesh)}
    if source.read_bytes() != source_bytes:
        raise ValueError('Scene declaration changed during export')
    destination = Path(destination)
    destination.mkdir(parents=False,exist_ok=False)
    (destination/'scene.json').write_bytes(source_bytes)
    for name, value in decks.items():
        with (destination/name).open('x',encoding='ascii') as output:
            output.write(value)
    record['files'] = [dict(path=name,bytes=(destination/name).stat().st_size,
                            sha256=sha256_file(destination/name)) for name in ('scene.json',*decks)]
    with (destination/'declared-scene.json').open('x',encoding='utf-8') as output:
        json.dump(record,output,indent=2,allow_nan=False)
        output.write('\n')
    return record


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path)
    parser.add_argument('destination',type=Path)
    args = parser.parse_args()
    record = export(args.source,args.destination)
    print(f"Exported {len(record['mesh']['nodes'])} physical nodes; numerical admission remains pending")
