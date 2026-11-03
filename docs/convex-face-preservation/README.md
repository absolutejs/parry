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
