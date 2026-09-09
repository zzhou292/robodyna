#!/usr/bin/env python3
"""Tiny standard-library CPU oracles; no solver, GPU, NumPy or source rewrite."""
import argparse
import ctypes as C
import json
import math
from pathlib import Path
import unittest

D = C.c_double


class State(C.Structure):
    _fields_ = [("off", D), ("smstr", D * 6), ("gstr", D * 8)]


class Input(C.Structure):
    _fields_ = [("x", D * 12), ("v", D * 12), ("omega", D * 12), ("frame", D * 9)] + [
        (name, D) for name in ("dt", "section_thickness", "current_thickness", "young",
                               "poisson", "density", "sound_speed", "shear_factor")] + [
        ("h", D * 3), ("srh", D * 3), ("ismstr", C.c_int), ("ithk", C.c_int)]


class Output(C.Structure):
    _fields_ = [("state", State), ("gradient", D * 4)] + [
        (name, D) for name in ("area", "vhx", "vhy", "thk0", "thk02", "shear_factor",
                               "vol0", "vol00")] + [
        ("material", D * 7), ("hg", D * 6), ("dstrain", D * 8),
        ("local_v", D * 12), ("local_omega", D * 12), ("gathered_off", D)]


def clone(value):
    return type(value).from_buffer_copy(bytes(value))


def fixture(mode=1):
    a = Input()
    a.x[:] = (-1, -.5, 0, 1, -.5, 0, 1, .5, 0, -1, .5, 0)
    a.frame[:] = (1, 0, 0, 0, 1, 0, 0, 0, 1)
    a.dt = .002
    a.section_thickness = .008
    a.current_thickness = .006
    a.young = 210e9
    a.poisson = .3
    a.density = 7800
    a.sound_speed = math.sqrt(a.young / ((1-a.poisson*a.poisson)*a.density))
    a.shear_factor = 5/6
    a.h[:] = (.11, .23, .37)
    a.srh[:] = (.41, .53, .67)
    a.ismstr = mode
    a.ithk = 0
    return a


def new_state():
    s = State()
    s.off = 1
    return s


def fill_affine(a, matrix, translation=(0, 0, 0)):
    for i in range(4):
        p = a.x[3*i:3*i+3]
        for k in range(3):
            a.v[3*i+k] = translation[k] + sum(matrix[k][j]*p[j] for j in range(3))


def rotate_x(p, angle):
    c, s = math.cos(angle), math.sin(angle)
    return (p[0], c*p[1]-s*p[2], s*p[1]+c*p[2])


def rotation(axis, angle):
    length = math.sqrt(sum(x*x for x in axis))
    u = [x/length for x in axis]
    c, s = math.cos(angle), math.sin(angle)
    cross = ((0, -u[2], u[1]), (u[2], 0, -u[0]), (-u[1], u[0], 0))
    return [[c*(i == j)+(1-c)*u[i]*u[j]+s*cross[i][j] for j in range(3)] for i in range(3)]


def apply_rotation(matrix, vector):
    return [sum(matrix[i][j]*vector[j] for j in range(3)) for i in range(3)]


class PhaseTests(unittest.TestCase):
    oracle = None

    def near(self, actual, expected, tolerance=2e-12):
        self.assertEqual(len(actual), len(expected))
        for a, e in zip(actual, expected):
            self.assertTrue(math.isfinite(a))
            self.assertLessEqual(abs(a-e), tolerance*max(1, abs(e)), (a, e))

    def call(self, inputs, states=None):
        states = states or [new_state() for _ in inputs]
        ia, sa = (Input*len(inputs))(*inputs), (State*len(inputs))(*states)
        before = bytes(sa)
        out = (Output*len(inputs))()
        self.assertEqual(self.oracle(len(inputs), ia, sa, out), 0)
        self.assertEqual(bytes(sa), before, "accepted state changed implicitly")
        return [clone(o) for o in out]

    def test_zero_reference_geometry_and_material_selection(self):
        for mode in (1, 2):
            a = fixture(mode)
            out = self.call([a])[0]
            self.assertEqual(out.state.off, 2 if mode == 1 else 1)
            self.assertEqual(out.gathered_off, 1)
            self.near(out.state.smstr, (2, 0, 2, 1, 0, 1))
            self.near(out.gradient, (-.5, .5, -1, -1))
            self.near((out.area, out.vhx, out.vhy), (2, 0, 0))
            self.near(out.dstrain, [0]*8)
            self.near(out.state.gstr, [0]*8)
            self.near(out.material, (a.density, a.young, a.poisson,
                a.young/(2*(1+a.poisson)), a.young/(1-a.poisson*a.poisson),
                a.young*a.poisson/(1-a.poisson*a.poisson), a.sound_speed))
            self.near(out.hg, list(a.h)+list(a.srh))

    def test_affine_membrane_engineering_shear_and_distorted_polygon(self):
        matrix = ((.23, -.17, 0), (.31, -.09, 0), (0, 0, 0))
        for distorted in (False, True):
            a = fixture(2)
            if distorted:
                a.x[:] = (-1, -.5, 0, 1.2, -.4, 0, .8, .7, 0, -.9, .4, 0)
            fill_affine(a, matrix, (2.3, -1.7, .4))
            state = new_state()
            state.gstr[:] = [(.2+i)*1e-4 for i in range(8)]
            out = self.call([a], [state])[0]
            expected = [a.dt*.23, a.dt*(-.09), a.dt*(.31-.17), 0, 0, 0, 0, 0]
            self.near(out.dstrain, expected)
            self.near(out.state.gstr, [s+d for s, d in zip(state.gstr, expected)])
            area = .5*sum(a.x[3*i]*a.x[3*((i+1)%4)+1]-a.x[3*i+1]*a.x[3*((i+1)%4)]
                            for i in range(4))
            self.near((out.area,), (area,))
            self.near(out.local_v, a.v)

    def test_curvature_and_mean_rotation_signs(self):
        a = fixture(2)
        for i in range(4):
            x, y = a.x[3*i:3*i+2]
            a.omega[3*i:3*i+3] = (.12 + .23*x-.31*y, -.17+.41*x+.53*y, .61)
        out = self.call([a])[0]
        self.near(out.dstrain, [0, 0, 0, -.12*a.dt, -.17*a.dt,
                                .41*a.dt, .31*a.dt, (.53-.23)*a.dt])
        self.near(out.local_omega, a.omega)

    def test_explicit_quadratic_term_is_retained(self):
        # Source-derived diagnostic: this is not an independent objectivity law.
        a = fixture(2)
        p, q = .43, -.27
        for i in range(4):
            x, y = a.x[3*i:3*i+2]
            a.v[3*i+2] = p*x+q*y
            a.omega[3*i:3*i+3] = (q, -p, 0)
        out = self.call([a])[0]
        self.near(out.dstrain, [-(a.dt*p)**2, -(a.dt*q)**2, 0, 0, 0, 0, 0, 0], 2e-14)

    def test_rigid_translation_and_in_plane_spin(self):
        a = fixture(2)
        fill_affine(a, ((0, -.7, 0), (.7, 0, 0), (0, 0, 0)), (3, -2, 1))
        for i in range(4):
            a.omega[3*i+2] = .7
        out = self.call([a])[0]
        self.near(out.dstrain, [0]*8)

    def test_fixed_frame_covariance(self):
        a = fixture(2)
        fill_affine(a, ((.2, -.3, 0), (.4, -.1, 0), (.03, -.04, 0)), (1, 2, 3))
        for i in range(4):
            a.omega[3*i:3*i+3] = (.1, -.2, .3)
        baseline = self.call([a])[0]
        rotated = clone(a)
        r = rotation((1, 2, 3), .83)
        for i in range(4):
            p = apply_rotation(r, a.x[3*i:3*i+3])
            rotated.x[3*i:3*i+3] = [p[j]+(4, -3, 2)[j] for j in range(3)]
            rotated.v[3*i:3*i+3] = apply_rotation(r, a.v[3*i:3*i+3])
            rotated.omega[3*i:3*i+3] = apply_rotation(r, a.omega[3*i:3*i+3])
        rotated.frame[:] = [r[j][i] for i in range(3) for j in range(3)]
        out = self.call([rotated])[0]
        self.near(out.dstrain, baseline.dstrain)
        self.near(out.state.smstr, baseline.state.smstr)
        self.near(out.local_v, baseline.local_v)
        self.near(out.local_omega, baseline.local_omega)

    def test_frozen_current_and_inherited_reference_policies(self):
        a = fixture(1)
        frozen = self.call([a])[0].state
        current = new_state()
        for i in range(4):
            a.x[3*i] *= 1.1
            a.x[3*i+1] *= .8
        fill_affine(a, ((.2, 0, 0), (0, -.3, 0), (0, 0, 0)))
        out_frozen = self.call([a], [frozen])[0]
        a.ismstr = 2
        out_current = self.call([a], [current])[0]
        out_inherited = self.call([a], [frozen])[0]
        self.near(out_frozen.dstrain, [.2*1.1*a.dt, -.3*.8*a.dt, 0, 0, 0, 0, 0, 0])
        self.near(out_current.dstrain, [.2*a.dt, -.3*a.dt, 0, 0, 0, 0, 0, 0])
        self.near(out_inherited.dstrain, out_frozen.dstrain)
        self.near((out_frozen.area, out_current.area, out_inherited.area), (2, 2*1.1*.8, 2))
        self.near(out_frozen.state.smstr, frozen.smstr)
        self.assertNotEqual(list(out_current.state.smstr), list(frozen.smstr))

    def test_thickness_snapshot_and_persistent_load_unload(self):
        a = fixture(1)
        state = self.call([a])[0].state
        total = 0
        for step, rate in enumerate((.2, .2, -.4, .1)):
            a.ithk = step % 2
            a.current_thickness = .006 + .0001*step
            fill_affine(a, ((rate, 0, 0), (0, 0, 0), (0, 0, 0)))
            out = self.call([a], [state])[0]
            total += rate*a.dt
            self.near(out.state.gstr, [total, 0, 0, 0, 0, 0, 0, 0])
            h = a.current_thickness if a.ithk else a.section_thickness
            self.near((out.thk0, out.thk02, out.shear_factor, out.vol0, out.vol00),
                      (h, h*h, a.shear_factor, 2*h, 2*a.section_thickness))
            state = clone(out.state)  # The only explicit commit in this test.

    def test_completed_interval_rigid_rotation_convergence(self):
        # Independent trajectory and phase construction, plus explicitly
        # source-derived expected finite-step residual. Never assert zero here.
        angle = .4
        for mode in (1, 2):
            norms = []
            for steps in (8, 16, 32):
                a = fixture(mode)
                reference = list(a.x)
                state = self.call([a])[0].state
                a.dt = 1/steps
                delta = angle/steps
                for step in range(1, steps+1):
                    t = step*delta
                    for i in range(4):
                        base = reference[3*i:3*i+3]
                        now, old = rotate_x(base, t), rotate_x(base, t-delta)
                        a.x[3*i:3*i+3] = now
                        a.v[3*i:3*i+3] = [(now[j]-old[j])/a.dt for j in range(3)]
                        a.omega[3*i:3*i+3] = (angle, 0, 0)
                    a.frame[:] = (1, 0, 0, 0, math.cos(t), math.sin(t),
                                  0, -math.sin(t), math.cos(t))
                    state = self.call([a], [state])[0].state
                expected = [0, steps*(math.cos(delta)**2-math.cos(delta)), 0,
                            steps*(math.sin(delta)-delta), 0, 0, 0, 0]
                self.near(state.gstr, expected, 3e-12)
                norms.append(math.sqrt(sum(x*x for x in state.gstr)))
            self.assertGreater(norms[0], 1e-5)
            self.assertLess(norms[1], .55*norms[0])
            self.assertLess(norms[2], .55*norms[1])

    def test_distinct_elements_permutation_and_last_element_failure_retry(self):
        a, b = fixture(1), fixture(2)
        b.young = 70e9
        b.poisson = .22
        b.density = 2700
        b.section_thickness = .013
        b.ithk = 1
        for i in range(4):
            b.x[3*i] *= .7
            b.x[3*i+1] *= 1.3
        fill_affine(a, ((.2, .1, 0), (0, -.1, 0), (0, 0, 0)))
        fill_affine(b, ((-.3, 0, 0), (.2, .4, 0), (0, 0, 0)))
        batch = self.call([a, b])
        for i, single in enumerate((a, b)):
            self.assertEqual(bytes(batch[i]), bytes(self.call([single])[0]))
        reverse = self.call([b, a])
        self.assertEqual(bytes(batch[0]), bytes(reverse[1]))
        self.assertEqual(bytes(batch[1]), bytes(reverse[0]))
        for native_failure in (False, True):
            bad = clone(b)
            if native_failure:
                bad.current_thickness = 1e200  # finite input; native square overflows
            else:
                bad.frame[0] = 2
            inputs, states = (Input*2)(a, bad), (State*2)(new_state(), new_state())
            out = (Output*2)()
            C.memset(C.addressof(out), 0x5a, C.sizeof(out))
            before = bytes(out)
            self.assertEqual(self.oracle(2, inputs, states, out), 2 if native_failure else 1)
            self.assertEqual(bytes(out), before)
            self.assertEqual(bytes(batch[1]), bytes(self.call([a, b])[1]))

    def test_invalid_domain_does_not_publish(self):
        a = fixture()
        bad_inputs = []
        for field, value in (("dt", 0), ("poisson", .5), ("ismstr", 11), ("ithk", 2),
                             ("young", float("nan")), ("section_thickness", -1)):
            b = clone(a)
            setattr(b, field, value)
            bad_inputs.append(b)
        b = clone(a); b.x[2] = .01; bad_inputs.append(b)
        b = clone(a); b.frame[8] = -1; bad_inputs.append(b)
        b = clone(a); b.x[6:9] = b.x[3:6]; bad_inputs.append(b)
        for b in bad_inputs:
            out = Output()
            C.memset(C.addressof(out), 0x6b, C.sizeof(out))
            before = bytes(out)
            state = new_state()
            self.assertEqual(self.oracle(1, C.byref(b), C.byref(state), C.byref(out)), 1)
            self.assertEqual(bytes(out), before)
        state = new_state(); state.off = 2  # missing retained geometry
        out = Output()
        self.assertEqual(self.oracle(1, C.byref(a), C.byref(state), C.byref(out)), 1)
        self.assertEqual(self.oracle(0, None, None, None), 1)
        self.assertEqual(self.oracle(3, None, None, None), 1)
        self.assertEqual(self.oracle(1, None, None, None), 1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--library", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args()
    if C.sizeof(State) != 15*8 or C.sizeof(Output) != 73*8:
        raise RuntimeError("Unexpected ctypes ABI layout")
    library = C.CDLL(str(args.library.resolve()))
    oracle = library.crash_shell_phase_native
    oracle.argtypes = [C.c_int, C.POINTER(Input), C.POINTER(State), C.POINTER(Output)]
    oracle.restype = C.c_int
    PhaseTests.oracle = staticmethod(oracle)
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(PhaseTests)
    ids = [test.id() for test in suite]
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    report = {"tests_run": result.testsRun, "failures": len(result.failures),
              "errors": len(result.errors), "skipped": len(result.skipped),
              "successful": result.wasSuccessful(), "test_ids": ids,
              "failure_details": [{"test": t.id(), "traceback": detail}
                                  for t, detail in result.failures + result.errors],
              "scope": "supplied-frame native geometry/strain phase; no material update or frame oracle"}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True)+"\n")
    raise SystemExit(0 if result.wasSuccessful() else 1)


if __name__ == "__main__":
    main()
