# Preserve noncoplanar convex-mesh facets

Baseline is the game's exact pinned Parry revision 6afbe26. A nine-vertex cube
with a shallow four-triangle top reduces the observed rock mass discrepancy:
normal similarity merges the raised top into a single flat polygon. The baseline
loses approximately 0.166% of volume, while an actually flat cube correctly merges
to six faces. This confirms the constructor issue independently of the generator.

The correction keeps the existing normal gate, then uses robust::orient3d to
require exact coplanarity of the represented coordinates before deleting an edge.
F32 values are widened exactly to f64. The existing pinned robust 1.2.0 workspace
dependency (already used for F64 hull dimensionality) is also enabled for parry3d.
No vertex, hull tolerance, mass override or contact setting is changed. Genuine
coplanar faces still merge, including an exactly represented affine transform.

Four integration tests pass in both scalar widths: shallow ridges across four
scales, flat cubes, affine coplanar cubes and the captured generated stone. The
captured stone retains 380 faces and its reconstructed mass matches input-triangle
mass exactly in both host builds. The failing baseline and successful logs remain.

The standard workspace's offline resolution lacks an optional encase index entry;
qualification uses the repository's existing reduced-dependency library manifests,
pointing directly to canonical src/lib.rs. Their dependency/lock changes are tracked.
The integration test also lives under the canonical crate test directories.

This is native source qualification only. A Wasm build, complete Rapier checks,
actual game candidate comparison, deterministic archive builds and a verified
published release remain necessary before production adoption. Exact preservation
may retain more faces than the former approximate merge and affect contact cost;
do not claim unchanged physics for formerly merged noncoplanar meshes.

Broader host library suites also pass: 85 F32 and 84 F64 tests. The cache/build
process is single-job and nice-prioritized. The captured game source is backed up
at hello-world commit 454265dd; it remains unadopted there.

## Cut-material support control

The full installed-archive game run found a settling regression in variant 7.
The captured convex fragment has 48 points; its f32 hull has 63 faces after exact
coplanarity preservation. The new focused support test confirms the selected
contact polygon includes the extreme supporting point in the captured initial
floor direction (zero projection deficit). Both f32 and f64 five-case suites
pass. This rules out a missing extreme vertex for that direction; it does not
qualify dynamic contact stability. The native one-body reproducer and release
hold are recorded in rapier3d's convex-face-preservation research directory.

## Analytic interior box-face experiment

The native one-fragment reduction sleeps in 7.62 s on an analytic plane but
remains awake at 20 s on box floors of two sizes. This branch excludes the failed
witness-coalescing policy. It adds a bounded exact-plane contact path only for
cuboid/convex-polyhedron pairs with no normal constraints. The entire transformed
hull must lie strictly within the face's tangential bounds, with clearance beyond
prediction plus penetration; its bounds center must be outside the face, and it
must not cross the opposite face. Other cases retain the general convex query.
The projected anchor remains in box-local coordinates; actual cuboid face IDs
and polyhedron vertex IDs retain contact identity across steps and shape order.

Fresh local-target libraries pass 89 f32 / 88 f64 tests before the additional
dispatch/identity test. The final five focused tests pass in each precision,
covering six faces, both orders, rotations, finite edges/corners, embedded cases,
separation and preserved contact data. Dynamic WebAssembly qualification remains
required; no publication or adoption is implied.

## Mandatory deepest support vertex

Full game research.2 qualification found five real regressions. A deterministic
96-orientation anisotropic-hull test proves a contact-generation hole: at
orientation 30, the most normal-aligned face provides zero contacts although
the true supporting vertex is 0.01 units inside the plane. The pre-fix failure
is preserved in interior-support-before.log.

The analytic box path now adds the exact supporting vertex when the selected
face does not already provide it, retaining its real vertex ID and the actual
box face ID. No tolerance, projection of the convex geometry, mass override or
sleep change is involved. The contact count remains bounded by five. All native
library tests pass: 91 f32 and 90 f64, including all 96 orientations. The next
WebAssembly candidate must re-run the five failing game cases before broader
qualification. This does not yet correct the separate generic halfspace path.
