# Scale-independent inertia reconstruction — 2026-10-28

Status: candidate native correction; not yet adopted or published in Rapier.

A matched rigid-body comparison exposed a mass-property aggregation discrepancy.
The existing cubic eigensolver uses an absolute cross-product threshold of 1e-20
when selecting a principal vector. Small non-isotropic tensors can fall below it;
near-repeated moments also make polynomial eigenvector recovery sensitive to
rounding. `baseline.log` reproduces up to 0.52741 relative tensor error for a
rotated two-sphere object at 0.001 length scale, versus 9.05e-5 at unit scale.
These are reconstruction errors, not a claim that every small game object errs
by that amount. The source dependency is glamx 0.3.1 `src/eigen3.rs`.

The canonical `MassProperties::with_inertia_matrix` candidate normalizes the
matrix and uses at most 32 Jacobi rotations in a double-precision workspace.
This handles repeated principal moments without the cubic polynomial root
recovery. Principal moments remain sorted, negative moments are still clamped
for subtraction, and quaternion signs are not considered physical differences.
There is no runtime JavaScript dependency patch or mass/energy clamp.

The canonical-library regression harness passes 10 tests, including the existing
four simplex witness tests and six new inertia tests. The latter exercise 960
tensors and 384 aggregation cases per precision (2,688 cases total): scales
1e-24 to 1e24, repeated/nearly repeated/anisotropic moments, 32 orientations and
six insertion orders. Maximum normalized reconstruction errors are 9.02e-7
(f32 tensors), 7.43e-7 (f32 aggregation), 3.08e-15 (f64 tensors), and 1.55e-15
(f64 aggregation). Zero and subtraction cases are also tested.

`regression.log` preserves an intermediate compile failure after removing a
needed VectorExt import; `regression-final.log` is the passing rerun. The larger
upstream unit target cannot resolve uncached `encase` offline. This is not a full
upstream suite claim. No graphical tests or GPU workloads were launched.

Commands (two compilation jobs, durable shared Cargo cache, isolated target):

```sh
cargo run --offline --manifest-path research/inertia-reconstruction/Cargo.toml
cargo test --locked --offline --manifest-path absolutejs/regression/Cargo.toml -- --nocapture
cargo check --offline --no-default-features --manifest-path research/inertia-reconstruction/Cargo.toml
```

Before adoption: validate no-std, rebuild and qualify the Rapier WASM engine,
repeat the game fixture and existing package/browser regressions, compare two
independent native builds, and release through the actual package repositories.
This does not solve the separate articulated COM/angular integration drift.
