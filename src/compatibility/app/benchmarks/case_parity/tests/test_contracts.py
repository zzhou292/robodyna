from copy import deepcopy
from pathlib import Path
import json
import tempfile
import unittest

from benchmarks.case_parity.artifacts import Artifacts
from benchmarks.case_parity.contracts import DOMAINS, compare_contracts, read_contract
from benchmarks.case_parity.performance import assess
from .support import Fixture


class ContractTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory(); self.addCleanup(tmp.cleanup)
        self.fixture = Fixture(tmp.name)

    def compare(self):
        f = self.fixture; files = Artifacts()
        return compare_contracts(read_contract(f.ref, f.root, files),
                                 read_contract(f.can, f.root, files))

    def test_every_domain_change_remains_incomparable_even_with_tiny_timing(self):
        f = self.fixture; initial = f.load(f.can)
        for domain in DOMAINS:
            with self.subTest(domain=domain):
                changed = deepcopy(initial)
                if domain == "recorded_time_grid":
                    changed["domains"][domain]["definition"]["requested_end_time_s"] = 9e-6
                elif domain == "output_work_precision":
                    changed["domains"][domain]["definition"]["fields"][0]["components"] = 4
                else:
                    changed["domains"][domain]["definition"]["changed"] = True
                f.can = f.write("candidate.json", changed)
                report = assess(f.request(), f.root)
                self.assertEqual(report["status"], "incomparable")
                self.assertIsNone(report["gpu_speedup"])
                self.assertTrue(any(x["domain"] == domain for x in report["mismatches"]))

    def test_unknown_on_both_sides_never_equals_declared_absence(self):
        f = self.fixture
        for name in ("ref", "can"):
            setattr(f, name, f.change(getattr(f, name), lambda d:
                d["domains"]["mass_inertia"].update(status="unknown", definition={"missing": "per-body ledger"})))
        result = assess(f.request(), f.root)
        self.assertEqual(result["status"], "incomparable")
        self.assertIn({"domain": "mass_inertia", "reason": "unresolved_evidence"}, result["mismatches"])

    def test_same_definition_different_evidence_locations_is_not_physics_change(self):
        f = self.fixture
        copied = f.write("second-evidence.json", f.load(f.declaration))
        f.can = f.change(f.can, lambda d: d["domains"]["population"].update(evidence=[copied]))
        self.assertEqual(self.compare(), [])

    def test_signed_zero_and_boolean_are_not_silently_normalized(self):
        f = self.fixture
        for value in (-0.0, False):
            f.ref = f.change(f.ref, lambda d: d["domains"]["population"].update(definition={"value": 0.0}))
            f.can = f.change(f.can, lambda d: d["domains"]["population"].update(definition={"value": value}))
            self.assertTrue(self.compare())

    def test_missing_extra_duplicate_and_nonfinite_json_reject(self):
        f = self.fixture
        original = Path(f.ref["path"]).read_bytes()
        mutations = [
            original.replace(b'"case_id": "reference"', b'"case_id": "reference", "case_id": "duplicate"'),
            original.replace(b'"scope": "synthetic"', b'"extra": 0, "scope": "synthetic"'),
            original.replace(b'"scope": "synthetic"', b'"scope": 1e400'),
            original.replace(b'"scope": "synthetic"', b'"scope": NaN'),
        ]
        for data in mutations:
            with self.subTest(data=data[:50]):
                pin = f.write("bad.json", data)
                with self.assertRaises(ValueError):
                    read_contract(pin, f.root)

    def test_stale_pin_and_bounded_json_fail_without_reading_unbounded_payload(self):
        f = self.fixture
        Path(f.ref["path"]).write_text("{}")
        with self.assertRaises(ValueError): read_contract(f.ref, f.root)
        large = f.write("large.json", b" " * ((1 << 20) + 1))
        with self.assertRaises(ValueError): read_contract(large, f.root)

    def test_missing_numerical_protocol_cannot_be_a_speed_claim(self):
        f = self.fixture
        f.ref = f.change(f.ref, lambda d: d.update(numerical_protocol=None))
        f.can = f.change(f.can, lambda d: d.update(numerical_protocol=None))
        result = assess(f.request(numerics=False), f.root)
        self.assertEqual(result["status"], "numerics_unqualified")
        self.assertIsNone(result["gpu_speedup"])

    def test_pass_boolean_is_not_numerical_evidence(self):
        f = self.fixture; request = f.load(f.request())
        request["numerics"] = True
        with self.assertRaises(ValueError): assess(f.write("request.json", request), f.root)


if __name__ == "__main__":
    unittest.main()
