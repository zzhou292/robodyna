#!/usr/bin/env python3
"""Small independent mode/work checks against the exact compiled native CHVIS3."""
import argparse
import ctypes
import hashlib
import json
import math
from pathlib import Path
import time
import unittest


def rectangle(a=1.0, b=.75, h=.02):
    # Boundary-integral b vectors are (-b,-a),(b,-a),(b,a),(-b,a).
    return [-b, b, -a, -a, 4*a*b, 0, 0, h, 2.1e11, .3,
            7800, 5200, 5/6, .04, .03, .02, .05, .06, .07, .001]


def soa(elements):
    return [element[c] for c in range(len(elements[0])) for element in elements]


def split(values, n, columns):
    return [[values[c*n+e] for c in range(columns)] for e in range(n)]


def mode(component, amplitude):
    result = [0.0]*12
    for node, sign in enumerate((1, -1, 1, -1)):
        result[3*node+component] = sign*amplitude
    return result


def coefficients(f, control):
    # Algebraic rectangle reduction of the cited native coefficient family;
    # invariants (null mode, force/couple/work and affine checks) are independent.
    px1, px2, py1, py2, area, _, _, h, young, nu, rho, sound, shf, h1, h2, h3, sr1, sr2, sr3, dt = f
    visc, linear, elastic = control
    s = shf/(3*(1+nu))
    k = [young*elastic*h1*h/4]*2 + [young*elastic*h2*s*h**3/(4*(px1**2+px2**2+py1**2+py2**2))]
    lin = [rho*linear*sr1*sound*h*math.sqrt(area)/(4*math.sqrt(2))]*2
    lin += [rho*linear*sr2*sound*math.sqrt(s)*h*h/(4*math.sqrt(2))]
    lin += [rho*linear*sr3*sound*.072169*area*h*h/(4*math.sqrt(2))]*2
    quad = [25*rho*visc*h1*h*math.sqrt(area)]*2
    quad += [25*rho*visc*h2*math.sqrt(s)*h*h]
    # Native rotational quadratic coefficient uses HELAS, not HVISC.
    quad += [25*rho*elastic*h3*.072169*area*h*h]*2
    return k, lin, quad


def rectangle_prediction(f, control, velocity, angular, old):
    signs = (1, -1, 1, -1)
    rates = [sum(signs[i]*velocity[3*i+j] for i in range(4)) for j in range(3)]
    rates += [sum(signs[i]*angular[3*i+j] for i in range(4)) for j in range(2)]
    stiffness, linear, quadratic = coefficients(f, control)
    history = [old[j]+stiffness[j]*f[19]*rates[j] for j in range(3)]
    response = [history[j]+linear[j]*rates[j]+quadratic[j]*abs(rates[j])*rates[j] for j in range(3)]
    rotations = [linear[j]*rates[j]+quadratic[j]*abs(rates[j])*rates[j] for j in (3,4)]
    history += rotations
    force = [-signs[i]*response[j] for i in range(4) for j in range(3)]
    couple = [-signs[i]*(rotations+[0])[j] for i in range(4) for j in range(3)]
    work = f[19]*sum(x*r for x,r in zip(response+rotations,rates))
    return history, force, couple, work


class Native:
    def __init__(self, path):
        self.library = ctypes.CDLL(str(path))
        self.fn = self.library.crash_chvis3_native
        pointer = ctypes.POINTER(ctypes.c_double)
        self.fn.argtypes = [ctypes.c_int, ctypes.c_int]+[pointer]*9
        self.fn.restype = ctypes.c_int

    def run(self, elements, velocity=None, angular=None, old=None, controls=(1,1,1), mode_value=1):
        n = len(elements)
        velocity = velocity if velocity is not None else [[0]*12 for _ in range(n)]
        angular = angular if angular is not None else [[0]*12 for _ in range(n)]
        old = old if old is not None else [[0]*5 for _ in range(n)]
        def array(values):
            return (ctypes.c_double*len(values))(*values)
        inputs = [array(controls),array(soa(elements)),array(soa(velocity)),array(soa(angular)),array(soa(old))]
        outputs = [array([987.0]*(n*c)) for c in (5,12,12,1)]
        status = self.fn(n, mode_value, *inputs, *outputs)
        return status, [split(value,n,c) for value,c in zip(outputs,(5,12,12,1))]


class NativeTests(unittest.TestCase):
    native = None

    def close(self, actual, expected, atol=1e-8):
        self.assertEqual(len(actual),len(expected))
        for a,b in zip(actual,expected):
            self.assertAlmostEqual(a,b,delta=atol+2e-12*abs(b))

    def predicted(self, f, controls, v, w, old):
        status, arrays = self.native.run([f],[v],[w],[old],controls)
        self.assertEqual(status,0)
        expected = rectangle_prediction(f,controls,v,w,old)
        for actual, reference in zip(arrays[:3],expected[:3]):
            self.close(actual[0],reference)
        self.close(arrays[3][0],[expected[3]])
        # Endpoint-force work independently equals minus restoring nodal work.
        nodal = sum(a*b for a,b in zip(arrays[1][0],v)) + sum(a*b for a,b in zip(arrays[2][0],w))
        self.close(arrays[3][0],[-f[19]*nodal])
        for values in arrays[1:3]:
            self.close([sum(values[0][3*i+j] for i in range(4)) for j in range(3)],[0]*3)
        return arrays

    def test_zero_and_retained_elastic_history(self):
        for controls in ((0,0,0),(1,1,1)):
            self.predicted(rectangle(),controls,[0]*12,[0]*12,[0]*5)
            result = self.predicted(rectangle(),controls,[0]*12,[0]*12,[2,-3,4,999,-888])
            self.close(result[0][0],[2,-3,4,0,0])
            self.close(result[3][0],[0])

    def test_all_five_modes_isolated_linear_quadratic_elastic(self):
        for controls in ((0,1,0),(1,0,0),(0,0,1),(1,1,1)):
            for component in range(5):
                for amplitude in (.002,-.003):
                    with self.subTest(controls=controls,component=component,amplitude=amplitude):
                        velocity = mode(component,amplitude) if component<3 else [0]*12
                        angular = mode(component-3,amplitude) if component>=3 else [0]*12
                        result = self.predicted(rectangle(),controls,velocity,angular,[0]*5)
                        self.assertGreaterEqual(result[3][0][0],-1e-12)

    def test_load_unload_reload_history_work(self):
        f=rectangle(); controls=(0,0,1); old=[0]*5
        work=[]
        for amplitude in (.003,-.001,-.002,.004):
            result=self.predicted(f,controls,mode(0,amplitude),[0]*12,old)
            old=result[0][0]; work.append(result[3][0][0])
        self.assertLess(work[1],0)  # Elastic unloading is not viscous dissipation.
        self.assertGreater(work[0],0)
        self.assertGreater(work[-1],0)

    def test_two_distinct_elements_preserve_soa_strides(self):
        fields=[rectangle(),rectangle(.6,.9,.035)]
        fields[1][8]=7e10;fields[1][10]=2700;fields[1][14]=.08
        v=[mode(0,.003),mode(2,-.004)]
        w=[mode(1,.002),mode(0,-.001)]
        old=[[1,-2,3,4,5],[-3,7,-1,9,2]]
        status,batch=self.native.run(fields,v,w,old)
        self.assertEqual(status,0)
        for e in range(2):
            status,single=self.native.run([fields[e]],[v[e]],[w[e]],[old[e]])
            self.assertEqual(status,0)
            for b,s in zip(batch,single):self.close(b[e],s[0])

    def test_corrected_translation_affine_and_uniform_branch_distinction(self):
        points=[(0,0),(2,0),(2.5,1),(.2,1.3)]
        area=.5*sum(points[i][0]*points[(i+1)%4][1]-points[(i+1)%4][0]*points[i][1] for i in range(4))
        b=[(.5*(points[(i+1)%4][1]-points[(i-1)%4][1]),
            .5*(points[(i-1)%4][0]-points[(i+1)%4][0])) for i in range(4)]
        f=rectangle();f[:5]=[b[0][0],b[1][0],b[0][1],b[1][1],area]
        f[5]=sum((1,-1,1,-1)[i]*points[i][0] for i in range(4))/area
        f[6]=sum((1,-1,1,-1)[i]*points[i][1] for i in range(4))/area
        for velocity in ([.004,-.002,.003]*4,
                         [value for x,y in points for value in (.002*x-.001*y,.003*y,.001*x+.004*y)]):
            status,result=self.native.run([f],[velocity],controls=(1,1,1),mode_value=2)
            self.assertEqual(status,0)
            self.close(result[0][0],[0]*5,1e-9);self.close(result[1][0],[0]*12,1e-9)
            self.close(result[3][0],[0],1e-9)
        status,uniform=self.native.run([f],[velocity],controls=(1,1,1),mode_value=1)
        self.assertEqual(status,0)
        self.assertGreater(max(abs(v) for v in uniform[1][0]),1e-4)

    def test_failure_does_not_publish_and_retry_is_clean(self):
        valid=rectangle()
        for index,value in ((4,0),(7,-1),(9,.5),(13,-.1),(19,0),(0,float('nan'))):
            altered=valid[:];altered[index]=value
            status,outputs=self.native.run([altered])
            self.assertEqual(status,1)
            for output in outputs:self.assertTrue(all(x==987 for x in output[0]))
        status,outputs=self.native.run([valid],mode_value=11)
        self.assertEqual(status,1)
        for output in outputs:self.assertTrue(all(x==987 for x in output[0]))
        status,a=self.native.run([valid],[mode(0,.001)])
        other_status,b=self.native.run([valid],[mode(0,.001)])
        self.assertEqual(status,0);self.assertEqual(other_status,0);self.assertEqual(a,b)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',required=True,type=Path)
    parser.add_argument('--report',required=True,type=Path)
    args=parser.parse_args()
    NativeTests.native=Native(args.library.resolve())
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(NativeTests)
    ids=[case.id() for case in suite]
    start=time.monotonic()
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    report=dict(tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),
                successful=result.wasSuccessful(),test_ids=ids,elapsed_seconds=time.monotonic()-start,
                library_sha256=hashlib.sha256(args.library.read_bytes()).hexdigest(),
                test_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                failure_details=[dict(test=str(test),traceback=t) for test,t in result.failures],
                error_details=[dict(test=str(test),traceback=t) for test,t in result.errors],
                scope='Exact native CHVIS3 with named synthetic coefficients; no CUDA/vehicle qualification')
    args.report.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
    if not result.wasSuccessful():raise SystemExit(1)


if __name__=='__main__':main()
