# Conservative 3D composite CCD filtering

The strict rotating-bar fixture starts entirely above y=0 and rotates 40 degrees
around a stationary centroid. Its tip ends below y=0. The original two-sided
query hits at fraction 0.74085635; the one-sided query reports separated because
its centroid/minimum-radius early-out never sees the tip's motion.

The candidate replaces that front-side shortcut with a whole-sweep AABB test.
The existing started-behind rule and initial-overlap fallback are unchanged.
The 2D implementation is unchanged. Generic GJK changes are qualified separately.

Let C(t) be the linearly interpolated COM and q(t) the normalized quaternion
lerp along the shortest arc. For a proxy vertex p, write r=p-local_center.
Its position is C(t)+R(q(t))r. Let rho be the maximum norm of these offsets,
and d=min(norm(q0-q1),norm(q0+q1)) for the normalized endpoint rotations over
the requested fraction. If theta is their rotation angle, d=2 sin(theta/4).
For any intermediate rotation, the displacement from the starting orientation
is at most 2 rho sin(theta/2), which is bounded by 2 rho d.

Therefore sweep the starting-orientation proxy box along the COM segment,
then expand by 2 rho d. The proxy inflation radius is already in the starting
box and does not rotate. This encloses the full arc in exact arithmetic.
The scalar expansion rounds outward and the local candidate box includes the
existing linear-slop margin. This is ordinary finite-precision conservative
geometry, not an interval-arithmetic proof for every extreme floating input.

Projecting the minimizing corner of that box onto a triangle's outward normal
provides a lower bound for every swept point. Only a strictly positive bound
permits the front-side early-out. A stationary centroid alone is insufficient.

The bound uses two linear vertex scans (start box and maximum radius), fixed-size
math and no heap allocation. Translation-only sweeps keep the tight box bound.
Independent sampled-box tests cover four axes, five rotation magnitudes, two
COM offsets, three clipped fractions and 101 samples each (12,120 sampled
boxes). The rotating-tip hit and a clipped interval before impact are also tested.
Selected-feature library tests pass in all four configurations: 3D/f32 (86),
3D/f64 (85), 2D/f32 (98), and 2D/f64 (98). Game tests remain pending. No performance claim follows from operation counts alone.

A separate fixture with clear endpoints at 87/93 degrees still exposes a miss
in the direct convex sweep itself. Fixing composite bounds does not claim to
repair that lower-level nonmonotonic-distance case. It remains recorded on
research/triangle-contact-conditioning and needs its own qualified correction.

The first f64 run failed only the newly introduced bitwise-equality assertion
for pure translation: differing addition order changed one bound coordinate by
8.88e-16. The failed log is retained. The corrected assertion checks containment
and tightness within eight machine epsilons; no production geometry was changed
in response. The final four logs and generated source-hash manifests describe
the same corrected source. Existing optional-feature cfg warnings are retained.
