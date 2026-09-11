#!/usr/bin/env python3
"""Bounded fixture-reader controls; no model import or mechanics execution."""
import argparse
import shutil
import tempfile
from pathlib import Path
import prepare_fixture as fixture

def controls(source):
    with tempfile.TemporaryDirectory(prefix='law90-fixture-control-') as tmp:
        root=Path(tmp)
        local=root/'input'
        local.mkdir()
        for name in ['manifest.json']+[key+'.bin' for key in fixture.SCHEMA]:
            shutil.copyfile(source/name,local/name)
        output=root/'Fixture.h'
        fixture.generate(local,output)
        before=output.read_bytes()
        fixture.generate(local,output,True)
        # A late source-array truncation rejects before overwriting output.
        last=local/'solid_source_line_u64.bin'
        data=last.read_bytes()
        last.write_bytes(data[:-1])
        try:
            fixture.generate(local,output)
        except ValueError:
            pass
        else:
            raise AssertionError('truncated late array admitted')
        assert output.read_bytes()==before
        last.write_bytes(data)
        data=bytearray(last.read_bytes())
        data[-1]^=1
        last.write_bytes(data)
        try:
            fixture.generate(local,output)
        except ValueError:
            pass
        else:
            raise AssertionError('changed array admitted')
        assert output.read_bytes()==before
        shutil.copyfile(source/'solid_source_line_u64.bin',last)
        manifest=local/'manifest.json'
        manifest.write_bytes(manifest.read_bytes()+b' ')
        try:
            fixture.generate(local,output)
        except ValueError:
            pass
        else:
            raise AssertionError('foreign manifest admitted')
        assert output.read_bytes()==before
        shutil.copyfile(source/'manifest.json',manifest)
        fixture.generate(local,output)
        assert output.read_bytes()==before
    print('PASS: fixture roundtrip, late size/hash/manifest rejection, preserved output and retry')
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',required=True,type=Path)
    controls(p.parse_args().input)
