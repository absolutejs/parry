# Separate direct-sweep rotational miss

This is not resolved by the composite swept-bound change at 48b51e0.
The strict fixture is retained on `research/triangle-contact-conditioning`,
`absolutejs/research/triangle-contact/composite_ccd.rs` (5fdfa8d).
A cuboid with half-extents (2, .001, .01), center (0, 1.999, 0), rotates
from 87 to 93 degrees about Z over a stationary triangle. Both endpoint
AABBs are above the plane, while the middle AABB crosses it. The direct
convex sweep reports separated before composite traversal is involved.

Source inspection identifies an unsafe assumption in the push-back loop of
`src/query/sweep_toi/sweep_toi.rs`: a positive final separation immediately
returns `Separated`. Rotating support points can have a nonmonotonic separation,
so endpoint separation alone cannot certify the entire interval. This explains
the analytic fixture, but is not yet a proven cause of the captured tree sink.

A future correction must certify the skipped interval using a conservative
motion bound or bounded interval search. Merely sampling the midpoint, using
more fixed substeps, or reporting the final pose as a hit does not establish
first-impact correctness. Retain bounded iteration/failure reporting, offset
centers of mass, clipped intervals, shape-order symmetry and scale tests. Test
both translating and rotating controls; measure query cost before adoption.
No implementation or release qualification is claimed by this note.
