# Signed modal work comparison

The production leaf and complete native SHOUR_CTL both evaluate 12 products in
Z1..4, X1..4, Y1..4 order, using 11 ordered additions and then multiplying by dt.
Production arithmetic, 0.3f, and MaximumPoissonRatio=double(0.48999f) are unchanged.

Write the exposed native modal force/rate as F_n,r_n and the GPU values as F_g,r_g.
Their measured differences are dF=F_g-F_n and dr=r_g-r_n. Exact product drift obeys

```
|F_g r_g - F_n r_n| <= |F_n| |dr| + |r_n| |dF| + |dF| |dr|.
D = |dt| sum over 12 terms of the right hand side.
```

For binary64 round-to-nearest unit roundoff u=epsilon/2, gamma13=13u/(1-13u).
Each term passes through one product, at most 11 additions, and the final dt
product. With finite normal intermediates (exact zeros allowed), each side's
forward error is bounded by gamma13 |dt| sum |F r|. The triangle inequality gives

```
|observed GPU work - observed native work|
  <= D + gamma13 |dt| sum (|F_g r_g| + |F_n r_n|).
```

The test helper computes an upper enclosure using wider long double and
nextafter toward positive infinity after each positive operation and measured
absolute difference. Products with a zero operand stay exactly zero; there is no absolute
floor. Compile-time precision/range assertions cover the checker. The helper
rejects nonfinite/subnormal products or partial sums and underflow-to-zero
products; this gate does not claim to qualify those arithmetic regimes.

The forward bound alone cannot reject reassociation or arbitrary errors that
happen to fit its budget. Therefore both work observations must also equal
binary64 replay of their own exposed fields in the required native order,
exactly (the signs of zero are equivalent). Modal fields separately retain their original 128-epsilon bounds.
Regressions accept cancellation with measured one-ulp modal drift but reject
bad work even inside the numerical envelope, a wrong fourth-mode factor, and
reordered modal accumulation. The existing fourth-mode native fixture remains.

The preserved diagnostic trace at clean commit 68288a74 contains 256 packets
(32 steps by 8 sound speeds). Its input packet has parameters9, velocities24,
incoming forces24, projections12, accepted history12; output is the existing
63-field packet. The original stopped gate2 at zero-based step4/packet1:

- Native work: 1.5251834391978320e-7 J; GPU: 1.5251834391978896e-7 J.
- Difference: 5.770411953169921e-21 J; old signed-relative bound: 4.334832053694504e-21 J.
- Native absolute product sum: 16.71570361730819 W.
- Measured modal drift bound: 5.551878956566724e-21 J.
- Product/addition/dt bound: 4.825121347473854e-20 J.
- Combined derived bound: 5.380309243130526e-20 J.

Completing the trace also finds old-comparator failure at step6/packet0. All 256
packets preserve exact own-field work identities and every other field bound.
An independent exact-rational audit obtains maximum error/bound 0.1696673866.
The trace is diagnostic evidence, not acceptance of the superseded comparator.
