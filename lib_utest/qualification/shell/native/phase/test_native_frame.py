#!/usr/bin/env python3
"""Exact native current-frame context; independent analytical CPU controls."""
import argparse
import ctypes as C
import json
import math
from pathlib import Path
import unittest

from test_native import Input, Output, State, clone, fixture, new_state, rotation, apply_rotation, rotate_x


class FrameInput(C.Structure):
    _fields_ = [("x", C.c_double * 12)]


class FrameOutput(C.Structure):
    _fields_ = [("frame", C.c_double * 9), ("direction_sentinels", C.c_double * 4)]


def input_frame(positions):
    result = FrameInput()
    result.x[:] = positions
    return result


def planar_bisector(points):
    # Independent angular construction of the symmetric frame for the selected
    # convex XY-plane polygons. This is source-frame semantics, not a new law.
    p = [points[3*i:3*i+2] for i in range(4)]
    xi = [p[1][j]-p[0][j]+p[2][j]-p[3][j] for j in range(2)]
    eta = [p[2][j]-p[1][j]+p[3][j]-p[0][j] for j in range(2)]
    a, b = math.atan2(xi[1], xi[0]), math.atan2(eta[1], eta[0])
    while b <= a:
        b += 2*math.pi
    if not 0 < b-a < math.pi:
        raise ValueError("Angular oracle domain")
    theta = .5*(a+b-math.pi/2)
    c, s = math.cos(theta), math.sin(theta)
    return [c, s, 0, -s, c, 0, 0, 0, 1]


class FrameTests(unittest.TestCase):
    frame_oracle = None
    phase_oracle = None

    def near(self, a, b, tolerance=3e-12):
        self.assertEqual(len(a), len(b))
        for actual, expected in zip(a, b):
            self.assertTrue(math.isfinite(actual))
            self.assertLessEqual(abs(actual-expected), tolerance*max(1, abs(expected)),
                                 (actual, expected))

    def frames(self, positions):
        inputs = (FrameInput*len(positions))(*[input_frame(p) for p in positions])
        out = (FrameOutput*len(positions))()
        before = bytes(inputs)
        self.assertEqual(self.frame_oracle(len(positions), inputs, out), 0)
        self.assertEqual(bytes(inputs), before)
        for value in out:
            self.assertEqual(list(value.direction_sentinels), [17.25, 17.25, -9.5, -9.5])
            e = [value.frame[3*i:3*i+3] for i in range(3)]
            for i in range(3):
                for j in range(3):
                    self.near([sum(e[i][k]*e[j][k] for k in range(3))], [int(i == j)])
            cross = [e[0][1]*e[1][2]-e[0][2]*e[1][1],
                     e[0][2]*e[1][0]-e[0][0]*e[1][2],
                     e[0][0]*e[1][1]-e[0][1]*e[1][0]]
            self.near(cross, e[2])
        return [clone(v) for v in out]

    def phase(self, a, state):
        out = Output()
        self.assertEqual(self.phase_oracle(1, C.byref(a), C.byref(state), C.byref(out)), 0)
        return out

    def test_rectangle_frame_and_unchanged_native_direction_buffers(self):
        a = fixture()
        out = self.frames([a.x])[0]
        self.near(out.frame, [1, 0, 0, 0, 1, 0, 0, 0, 1])

    def test_distorted_angular_frame_and_global_rotation_covariance(self):
        polygons = [list(fixture().x),
                    [-1, -.5, 0, 1.2, -.4, 0, .8, .7, 0, -.9, .4, 0],
                    [-1, -.5, 0, 1, -.5, 0, 1.3, .7, 0, -.7, .7, 0]]
        for polygon in polygons:
            expected = planar_bisector(polygon)
            self.near(self.frames([polygon])[0].frame, expected)
            for scale in (.25, 5):
                r = rotation((1, 2, -3), .83)
                moved = []
                for i in range(4):
                    p = apply_rotation(r, [scale*x for x in polygon[3*i:3*i+3]])
                    moved.extend([p[j]+(4, -2, 3)[j] for j in range(3)])
                rotated = []
                for i in range(3):
                    rotated.extend(apply_rotation(r, expected[3*i:3*i+3]))
                self.near(self.frames([moved])[0].frame, rotated)

    def test_distinct_batch_and_permutation(self):
        p = list(fixture().x)
        q = [-1, -.5, 0, 1.2, -.4, 0, .8, .7, 0, -.9, .4, 0]
        batch = self.frames([p, q])
        reverse = self.frames([q, p])
        self.assertEqual(bytes(batch[0]), bytes(reverse[1]))
        self.assertEqual(bytes(batch[1]), bytes(reverse[0]))
        self.assertEqual(bytes(batch[0]), bytes(self.frames([p])[0]))
        self.assertEqual(bytes(batch[1]), bytes(self.frames([q])[0]))

    def test_native_frame_composed_with_affine_phase(self):
        a = fixture(2)
        a.x[:] = (-1, -.5, 0, 1.2, -.4, 0, .8, .7, 0, -.9, .4, 0)
        a.frame[:] = self.frames([a.x])[0].frame
        l = ((.2, -.1), (.3, -.4))
        for i in range(4):
            x, y = a.x[3*i:3*i+2]
            a.v[3*i:3*i+3] = (l[0][0]*x+l[0][1]*y,
                                     l[1][0]*x+l[1][1]*y, 0)
        local = [[sum(a.frame[3*i+k]*l[k][m]*a.frame[3*j+m]
                      for k in range(2) for m in range(2)) for j in range(2)] for i in range(2)]
        out = self.phase(a, new_state())
        self.near(out.dstrain, [local[0][0]*a.dt, local[1][1]*a.dt,
                                (local[0][1]+local[1][0])*a.dt, 0, 0, 0, 0, 0])
        self.near([out.dstrain[0]+out.dstrain[1]], [(l[0][0]+l[1][1])*a.dt])

    def test_native_frame_phase_rigid_secant_trajectory_retains_known_residual(self):
        # Context qualification only: full native CNVEC3/CORTDIR3 does not
        # cancel the admitted IHBE1 finite-step strain residual.
        angle = .4
        for mode in (1, 2):
            norms = []
            for steps in (8, 16, 32):
                a = fixture(mode)
                reference = list(a.x)
                a.frame[:] = self.frames([a.x])[0].frame
                state = clone(self.phase(a, new_state()).state)
                a.dt = 1/steps
                delta = angle/steps
                for step in range(1, steps+1):
                    t = delta*step
                    for i in range(4):
                        p = reference[3*i:3*i+3]
                        now, old = rotate_x(p, t), rotate_x(p, t-delta)
                        a.x[3*i:3*i+3] = now
                        a.v[3*i:3*i+3] = [(now[j]-old[j])/a.dt for j in range(3)]
                        a.omega[3*i:3*i+3] = (angle, 0, 0)
                    a.frame[:] = self.frames([a.x])[0].frame
                    self.near(a.frame, [1, 0, 0, 0, math.cos(t), math.sin(t),
                                        0, -math.sin(t), math.cos(t)])
                    state = clone(self.phase(a, state).state)
                self.near(state.gstr, [0, steps*(math.cos(delta)**2-math.cos(delta)), 0,
                                       steps*(math.sin(delta)-delta), 0, 0, 0, 0], 3e-12)
                norms.append(math.sqrt(sum(x*x for x in state.gstr)))
            self.assertGreater(norms[0], 1e-5)
            self.assertLess(norms[1], .55*norms[0])
            self.assertLess(norms[2], .55*norms[1])

    def test_invalid_finite_planarity_and_degenerate_inputs_do_not_publish(self):
        p = list(fixture().x)
        bad = []
        for index, value in ((0, float("nan")), (4, float("inf")), (2, .01)):
            q = p.copy(); q[index] = value; bad.append(q)
        q = p.copy(); q[6:9] = q[3:6]; bad.append(q)
        q = p.copy(); q[6:9] = [0, -.6, 0]; bad.append(q)
        q = p.copy(); q[3:6], q[6:9] = q[6:9], q[3:6]; bad.append(q)
        bad.append([0]*12)
        bad.append([x for i in range(4) for x in (float(i), 0, 0)])
        for q in bad:
            inputs = (FrameInput*2)(input_frame(p), input_frame(q))
            out = (FrameOutput*2)()
            C.memset(C.addressof(out), 0x5b, C.sizeof(out))
            before = bytes(out)
            self.assertEqual(self.frame_oracle(2, inputs, out), 1)
            self.assertEqual(bytes(out), before)
        for n in (0, 1, 3):
            self.assertEqual(self.frame_oracle(n, None, None), 1)

    def test_invalid_native_output_is_atomic_and_retry_is_clean(self):
        p = list(fixture().x)
        baseline = self.frames([p, p])
        huge = [x*1e200 for x in p]
        inputs = (FrameInput*2)(input_frame(p), input_frame(huge))
        out = (FrameOutput*2)()
        C.memset(C.addressof(out), 0x3d, C.sizeof(out))
        before = bytes(out)
        self.assertEqual(self.frame_oracle(2, inputs, out), 2)
        self.assertEqual(bytes(out), before)
        self.assertEqual([bytes(x) for x in self.frames([p, p])], [bytes(x) for x in baseline])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--phase-library", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    if C.sizeof(FrameInput) != 12*8 or C.sizeof(FrameOutput) != 13*8:
        raise RuntimeError("Unexpected frame ctypes ABI")
    frame_library, phase_library = C.CDLL(str(args.library.resolve())), C.CDLL(str(args.phase_library.resolve()))
    frame = frame_library.crash_shell_frame_native
    frame.argtypes = [C.c_int, C.POINTER(FrameInput), C.POINTER(FrameOutput)]
    frame.restype = C.c_int
    phase = phase_library.crash_shell_phase_native
    phase.argtypes = [C.c_int, C.POINTER(Input), C.POINTER(State), C.POINTER(Output)]
    phase.restype = C.c_int
    FrameTests.frame_oracle, FrameTests.phase_oracle = staticmethod(frame), staticmethod(phase)
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(FrameTests)
    ids = [test.id() for test in suite]
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    report = dict(tests_run=result.testsRun, failures=len(result.failures), errors=len(result.errors),
                  skipped=len(result.skipped), successful=result.wasSuccessful(), test_ids=ids,
                  production_objectivity_qualified=False, native_startup_qualified=False,
                  yaris_formulation_mapping_qualified=False,
                  failure_details=[dict(test=t.id(), traceback=detail) for t, detail in result.failures+result.errors],
                  scope="full exact current-frame routines in explicit IREP0/IDRAPE0/ISHFRAM0 context; no objectivity or startup qualification")
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True)+"\n")
    raise SystemExit(0 if result.wasSuccessful() else 1)


if __name__ == "__main__":
    main()
