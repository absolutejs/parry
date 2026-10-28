# Fixed-plane interval bound (research)

For a fixed world unit direction n, let g(t) be min_B dot(n, p_B(t)) minus
max_A dot(n, p_A(t)). A positive g separates the convex core point clouds.
No GJK convergence claim is required for this plane: project the actual vertices.
A poor suggested direction may fail to separate and must not certify a skipped
interval. The initial prototype incorrectly relied on a witness direction alone.

At the current t, let v_min be the minimum projected point velocity on B minus
the maximum projected point velocity on A. For every vertex pair, the initial
projected separation is at least g and its derivative is at least v_min.
If the sum of the shapes' projected acceleration bounds is M, Taylor's bound
therefore gives g(t+h) >= g(t) + v_min*h - M*h*h/2. This applies to every h in
the skipped interval, including changing extremal vertices. Using
c = max(0, -v_min) is looser but makes the bound monotonically decreasing.

For quaternion nlerp u(t)=a+t*d, q=u/|u|, let m be a lower bound on |u| on the
whole requested interval. Then |q'| <= |d|/m and the triangle inequality applied
to q'' gives |q''| <= 6|d|²/m². For rotation R(q)v=q*v*conjugate(q), the second
derivative norm is at most (2|q''|+2|q'|²)|v|, hence 14|v||d|²/m². The radius is
measured around local_center, not the local origin. COM translation is linear
and contributes no acceleration. Add both shapes' bounds for M. The constant
14 is deliberately loose, not an optimal curvature estimate.

The positive root of margin-c*h-M*h*h/2 is evaluated without cancellation as
2*margin/(c+sqrt(c*c+2*M*margin)). A finite capped advance within that root
cannot skip a collision under the mathematical bound. When the root exceeds
the remaining interval, that interval is separated on this fixed plane. If
no positive margin exists or progress/capped iteration fails, report Failed;
never translate uncertainty into Separated. Existing distance/target tolerance
still controls a reported Hit; the bound must not enlarge collision tolerance.

These equations concern ideal real arithmetic and normalized linear interpolation.
Floating-point implementation still needs explicit error allowances and empirical
qualification; this note is not a machine-checked interval proof. The first f32
allowance was too coarse for a 1e-5 slop and stopped early. Double-precision local
bound arithmetic is being investigated. Endpoint quaternion normalization must
not silently reparameterize the supplied motion: compute the minimum norm of the
actual affine quaternion segment, including slightly nonunit stored endpoints.

Remaining gates: exact first-impact plane oracle, all scale/order controls,
quaternion sign invariance, clipped time, offset COM, translating/rotating pairs,
grazing/no-hit cases, bounded work/cost, native feature configurations, and real
game captured replay. Composite traversal and one-sided filtering are separate.
No production integration or release qualification is claimed.
