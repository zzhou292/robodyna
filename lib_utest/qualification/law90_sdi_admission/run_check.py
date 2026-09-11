#!/usr/bin/env python3
"""Execute native SDI observations and validate source/default/unit closure."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess


def require(condition, message):
    if not condition:
        raise ValueError(message)


def fields(rows):
    result = {r['name']: r for r in rows}
    require(len(result) == len(rows), 'duplicate observation field')
    return result


def check_target(packet, expected_curve):
    require(packet['material_id'] == 2000063 and packet['curve_id'] == 2100015, 'target identity')
    require(packet['curve_working_xy'] == expected_curve, 'original curve changed')
    f = fields(packet['fields'])
    require(f['LSD_MAT83_ED']['available'] and f['LSD_MAT83_ED']['effective_native_value'] == 20000,
            'KCON raw scalar changed')
    require(f['LSD_MAT83_ED']['dimension_available'] and
            f['LSD_MAT83_ED']['dimensions_lmt'] == [-1, 1, -2], 'target Kcont pressure dimension')
    require(f['FscaleL']['dimensions_lmt'] == [-1, 1, -2], 'native curve scale pressure dimension')
    require(f['MAT_TFLAG']['effective_native_value'] == 2, 'TFLAG2 lost')
    require(f['NL']['effective_native_value'] == 1 and f['Ismooth']['effective_native_value'] == 1,
            'converter single-curve smooth input')
    require(not f['DAMP']['available'], 'source DAMP was copied into LAW90')
    native = packet['native_preparation_si']
    require(len(native) == 33 and all(math.isfinite(x) for x in native), 'native packet shape')
    require(native[5] == 20e9 and native[7] == 15e6, 'native target pressure interpretation')
    require(native[14:16] == [0, 0] and native[17:23] == [0, 2, 0, 2, 10, 3], 'native flags/history')
    expected_flag = 1 if f['Hys']['effective_native_value'] == 0 else 2
    require(native[16] == expected_flag, 'native Hys pre-default classifier')
    require(native[8:11] == [1, 1, 1], 'native Hys/Shape/Alpha defaults')
    require(packet['qualified_positive_hys_profile'] == (expected_flag == 2), 'admission label')
    return expected_flag


def check_observation(data, expected_curve, explicit_hu=False):
    require(data['schema'] == 'law90.native_sdi_observation.v1' and data['export_format'] == 2026,
            'native observation schema/format')
    source = fields(data['source_fields'])
    require(source['KCON']['available'] and source['KCON']['effective_native_value'] == 20000,
            'source KCON literal')
    require(source['KCON']['dimensions_lmt'] == [0, 1, -2], 'source KCON stiffness dimension')
    result = {phase: check_target(data[phase], expected_curve) for phase in ('direct', 'exported_reread')}
    if explicit_hu:
        require(source['HU']['available'] and source['HU']['effective_native_value'] == 1,
                'explicit HU control missing')
        require(result == {'direct': 2, 'exported_reread': 2}, 'explicit HU control classifier')
    return result


def run(executable, pinned, fixture, output):
    output.mkdir(parents=True, exist_ok=False)
    inputs = json.loads((fixture / 'manifest.json').read_text())
    env = dict(os.environ)
    env.update(HM_MSG_DIR=str(pinned / 'hm_cfg_files/messages'),
               HM_MV_CFG_DIR=str(pinned / 'hm_cfg_files/config/CFG'),
               HM_MV_UNITS_DIR=str(pinned / 'hm_cfg_files/config/CFG/UNITS'))
    results = {}
    for name, record in inputs['files'].items():
        source = fixture / record['file']
        require(hashlib.sha256(source.read_bytes()).hexdigest() == record['sha256'], 'fixture changed')
        report = output / (name + '.json')
        exported = output / (name + '.rad')
        with (output / (name + '.log')).open('xb') as log:
            subprocess.run([str(executable), str(source), str(exported), str(report)],
                           env=env, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=120)
        d = json.loads(report.read_text())
        result = check_observation(d, inputs['curve_working_xy'], name == 'explicit_hu_one')
        results[name] = {'classifiers': result, 'export_sha256': hashlib.sha256(exported.read_bytes()).hexdigest(),
                         'observation_sha256': hashlib.sha256(report.read_bytes()).hexdigest()}
    baseline = json.loads((output / 'original.json').read_text())
    damping = json.loads((output / 'source_damp_control.json').read_text())
    require(fields(damping['source_fields'])['DAMP']['effective_native_value'] == .2, 'DAMP control unread')
    for phase in ('direct', 'exported_reread'):
        require(baseline[phase] == damping[phase], 'source DAMP altered native LAW90 material')
    receipt = {'schema': 'law90.sdi_admission_result.v1', 'observations': results,
               'direct_original_matches_qualified_iflag2': results['original']['classifiers']['direct'] == 2,
               'exported_original_matches_qualified_iflag2': results['original']['classifiers']['exported_reread'] == 2,
               'app_admission_implemented': False}
    (output / 'result.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    for arg in ('executable', 'pinned', 'fixture', 'output'):
        p.add_argument('--' + arg, type=Path, required=True)
    run(**vars(p.parse_args()))
