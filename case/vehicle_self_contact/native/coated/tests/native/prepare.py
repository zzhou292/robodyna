"""Independent native reader, IN24 classification and six-word surface sorting."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json

ROOT = Path(__file__).resolve().parent


def generate(tl_root):
    helper = tl_root / 'lib_utest/qualification/radioss_type25_selection/native/Sources.py'
    spec = importlib.util.spec_from_file_location('coated_reference_sources', helper)
    source = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(source)
    donors = {}
    for entry in json.loads((ROOT / 'source-manifest.json').read_text())['files']:
        data = (ROOT / entry['path']).read_bytes()
        assert len(data) == entry['bytes']
        assert hashlib.sha256(data).hexdigest() == entry['sha256']
        assert hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest() == entry['git_blob']
        donors[Path(entry['path']).name] = data.decode()

    reader = donors['hm_read_solid.F']
    start = '          ISOLNOD(I)=8\n'
    end = '          ENDIF\n'
    assert reader.count(start) == 1
    tail = reader.split(start, 1)[1]
    h8 = start + tail.split(end, 1)[0] + end
    assert 'CHECKVOLUME_8N' in h8 and 'IXS(9,I) = IC8' in h8
    marker = 'C      READING PENTA6 INPUTS IN HM STRUCTURE'
    stop = 'C      READING TETRA10 INPUTS IN HM STRUCTURE'
    assert reader.count(marker) == reader.count(stop) == 1
    penta = reader.split(marker, 1)[1].split(stop, 1)[0]
    start = '        IXS(9,I)=IXS(5,I)\n'
    end = '        ISOLNOD(I)=6\n'
    assert penta.count(start) == penta.count(end) == 1
    penta = start + penta.split(start, 1)[1].split(end, 1)[0] + end
    assert 'CHECKVOLUME' not in penta

    surface_lines = donors['i25surfi.F'].splitlines(keepends=True)
    starts = [i for i, line in enumerate(surface_lines)
              if line.strip() == 'IRECTMP_SAV(1:6,1:NRTM) =  IRECTMP(1:6,1:NRTM)']
    ends = [i for i, line in enumerate(surface_lines)
            if line.strip() == 'CALL MY_ORDERS( MODE, WORK, IRECTMP, INDEX, NRTM , 6)']
    assert len(starts) == len(ends) == 1 and starts[0] < ends[0]
    ordering = ''.join(surface_lines[starts[0]:ends[0] + 1])
    assert 'IRECTMP(5,I)==0.OR.IRECTMP(5,I)==1' in ordering
    routines = '\n'.join(source.routine(donors['i24surfi.F'], name)
                          for name in ('IN24COQ_SOL3', 'SEG_INS'))
    constants = source.constants(donors['constant_mod.F'],
                                 [routines, donors['checksvolume.F'], h8, penta])
    constants = constants.replace('selection_constants', 'coated_source_constants')
    outputs = {'Constants.F90': constants, 'Classification.F': routines,
               'ChecksVolume.F': donors['checksvolume.F'], 'my_orders.c': donors['my_orders.c']}
    for name, substitutions in [('Reader.F', {'H8_READER': h8, 'PENTA_READER': penta}),
                                ('Order.F', {'SURFACE_ORDER': ordering})]:
        text = (ROOT / (name + '.in')).read_text()
        for tag, block in substitutions.items():
            assert text.count('@' + tag + '@') == 1
            text = text.replace('@' + tag + '@', block)
        outputs[name] = text
    outputs['Roles.F90'] = (ROOT / 'Roles.F90').read_text()
    outputs['Element.F90'] = ('module element_mod\n implicit none\n'
                              ' integer,parameter::nixs=11,nixc=7,nixtg=6\nend module\n')
    outputs['implicit_f.inc'] = ('      USE ISO_C_BINDING\n      USE COATED_SOURCE_CONSTANTS\n'
                                 '      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n')
    outputs['com04_c.inc'] = (
        '      INTEGER NUMELC,NUMELTG,NUMELS,NUMELS8,NUMELS10,\n'
        '     . NUMELS16,NUMELS20\n'
        '      COMMON /COATED_REFERENCE_COUNTS/ NUMELC,NUMELTG,NUMELS,\n'
        '     . NUMELS8,NUMELS10,NUMELS16,NUMELS20\n')
    return outputs


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--tl-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    for name, text in generate(args.tl_root).items():
        path = args.output / name
        if args.check:
            assert path.read_text() == text
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
    print('Pinned reader blocks, complete IN24/SEG_INS and original surface ordering prepared')
