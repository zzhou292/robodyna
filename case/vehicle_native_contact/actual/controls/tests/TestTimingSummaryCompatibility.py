"""Reuse the qualified closure fixture; no renderer or solver is invoked."""
import argparse
import copy
from pathlib import Path
import sys
import unittest
parser=argparse.ArgumentParser()
parser.add_argument('--postprocess-root', type=Path, required=True)
args, remainder=parser.parse_known_args()
sys.path.insert(0,str(args.postprocess_root))
from viewer.postprocess import test_native_vehicle_run as fixtures
from viewer.postprocess.lifecycle import closed_run

class TimingSummaryCompatibility(unittest.TestCase):
    def setUp(self):
        self.fixture=fixtures.NativeVehicleCompletionTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)

    def test_old_and_new_native_summaries_keep_identical_archive_authority(self):
        f=self.fixture
        baseline=closed_run(f.launch)
        archive_before={p:p.read_bytes() for p in (f.root/'archive').iterdir()}
        for enabled in (False, True):
            timing=dict(enabled=enabled, counter_saturated=False, clock_failures=0,
                        backward_samples=0, total=[], last_attempt=[])
            f.summary.update(stage_profiling_enabled=enabled, mechanics_stage_timing=timing)
            f.publish()
            after=closed_run(f.launch)
            self.assertEqual(after[1:],baseline[1:])
            self.assertEqual(after[0]['mechanics_stage_timing'],timing)
            for field in ('schema','accepted_intervals','actual_completed_time_s','viewer_input_sha256','archive_manifest_sha256'):
                self.assertEqual(after[0][field],baseline[0][field])
            self.assertEqual(archive_before,{p:p.read_bytes() for p in (f.root/'archive').iterdir()})

    def test_timing_observations_cannot_authorize_an_incomplete_or_failed_run(self):
        f=self.fixture
        f.summary.update(stage_profiling_enabled=True,mechanics_stage_timing=dict(enabled=True,total=[],last_attempt=[]))
        original=copy.deepcopy(f.summary)
        f.summary['accepted_intervals']=17
        f.publish()
        with self.assertRaises(ValueError):closed_run(f.launch)
        f.summary=original
        f.prefix(1)
        with self.assertRaises(ValueError):closed_run(f.launch)

if __name__=='__main__':unittest.main(argv=[sys.argv[0],*remainder])
