# Triangle contact coordinate conditioning (private)

Base Parry source: c566aabd77be8ac7a427c2b704860c47e34657b9, the exact
revision shipped by @absolutejs/rapier3d 0.16.2. No canonical dependency change.

The geometry fixtures are exact serde exports from the real game's isolated
heightfield candidate, variant 3/root 15/1920 Hz, before and after step 2404.
Original snapshots, standalone triangle-convex reduction and an independent
f64 plane-separation oracle are backed up in absolutejs/rapier3d branch
research/composite-contact-reduction at 0d8f82b. They demonstrate a 319 mm
false manifold despite a plane-separation bound below 0.000005 mm.

## Candidate design

Keep each 3D heightfield triangle's contact coordinates relative to its first
vertex, using ContactManifold's existing explicit subshape pose. Relative
triangle/convex queries then operate near the triangle instead of at a large
heightfield-local offset. Feature IDs and triangle ordering remain unchanged.
Translations preserve pseudo-normal directions. Handle either collider order;
retain the same frame through contact reuse and serialization. If a restored
manifold has the historical identity frame, convert its local points before
querying and set the frame explicitly. Do not clamp depths, discard troublesome
contacts, increase tolerances or change the game solver budget.

This is a narrow heightfield conditioning candidate, not a claim to fix all
GJK degeneracy, generic triangle queries or triangulated terrain. Existing
subshape metadata allocates one small frame record per active triangle manifold;
measure CPU/memory impact before adoption. The game's heightfield candidate
remains rejected until broader physical comparisons and release checks pass.

## Qualification

First run the four exact before/after, forward/reverse shape-order queries
against unchanged source. Repeat each query after exact snapshot roundtrip to
exercise cached frames. Original source should reproduce the before-state
failure; a candidate must pass the original limits. Expand to frame migration,
rotated collider poses, pseudo-normal constraints, unrelated shape controls,
2D/f64 compilation, whole-tree WASM matrices and deterministic snapshot replay
before any publication. Native arithmetic is not a substitute for WASM tests.

Use offline Cargo, at most one compiler job and no native GPU. All fixtures,
source, logs and release evidence stay in durable checkouts under ~/alex.

## Baseline result and integration constraint

The unchanged source reproduces the false contact exactly: 3/4 focused tests
pass, and before/heightfield-first fails at -31.968266 centimetres. Reversed
shape order passes already, so symmetry is a regression control. The initial
harness omitted Parry's `alloc` feature and did not compile; that log is kept.
The corrected harness explicitly enables the same std+alloc feature combination
used by Rapier and records its exact dependency lock.

Rapier's native solver and contact clustering compose existing subshape poses.
However, its current TypeScript raw local-contact-point bindings return stored
subshape-local coordinates without exposing that pose. Any release adopting
translated heightfield frames must also make collider-space diagnostic access
explicit and tested. Otherwise physics may be correct while contact inspection
silently reports wrong positions. This is a release blocker, not a reason to
remove the frame or hide inaccurate observations.
