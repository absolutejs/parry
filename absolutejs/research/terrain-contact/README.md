# Prior-direction terrain contact regression

Base: published Parry 8c68a62, shipped in rapier3d 0.17.0. The fixture comes
from native onset reduction at engine 18bb4a9: step 877 recycle-base pose,
triangle 999, actual convex topology and previous cached direction.
The expected separation is about 0.01 cm within a 2 cm prediction distance.
Original code returns an empty manifold with normal X; baseline.log fails.

The trace identified exactly duplicated simplex vertices 1 and 3. The exact
Decimal oracle confirms zero tetrahedral volume and an affine plane separated
from the origin by 0.010006285696 cm. A rounded normalized face-plane test
had admitted an existing vertex as a fourth point, producing false Intersection
and failed EPA fallback. The production fix rejects an exactly repeated CSO
position before that plane test. It adds no epsilon and changes no solver,
recycling, prediction or iteration settings. Temporary GJK traces are removed.

Fresh qualification uses this checkout's .cache/source-qualified target:
- Captured contact and 72 scale/order/origin/clear/repeated-query controls pass.
- Source witness regression runs in both f32 and f64 (including distinct
  original witnesses for the same CSO position).
- Selected-feature library tests: 85 f32 3D, 84 f64 3D, 97 each 2D precision.
- Existing oak, heightfield and sweep control tests pass.
- All 432 rotational matrix records exactly equal delivered 0.17.0 records.
- Both 3D no-std/alloc checks pass.

inherited-cache-observations is explicitly INVALID qualification evidence:
a shared worktree target omitted the new source test. All gates above were
rerun in a fresh target and the new test's execution checked explicitly.
Do not share Cargo targets across source worktrees for qualification.

These native checks are not whole-game acceptance. A private engine build and
actual captured-tree replay follow; canonical game remains published 0.17.0.
