# AbsoluteJS Parry contact correction

This source fork starts at upstream Parry 0.31.1 (`3383f51cbbe9af70565427e7a66c605e1557c1fc`). It preserves prior simplex support points when a 3D tetrahedron is reduced to a face. Three direct vertex assignments become swaps through the existing bookkeeping helper. No contact tolerance, iteration limit, shape geometry or simulation policy changes.

GJK can return the previous simplex when its distance upper bound regresses. Previously, tetrahedron face reduction overwrote a support vertex while retaining its old barycentric weight and index. The returned witnesses could combine different simplex states and report large false penetrations. A captured floor/wood case returned -14.719921 cm while the shapes were separated. The correction reports +0.000338470 cm in the same single-precision reduction.

Regression tests cover all three replaced-face vertices and the captured simplex in f32/f64. The standalone `absolutejs/regression` harness depends on the actual canonical crates and avoids compiling upstream graphical demo dependencies. Run with two jobs and a durable target/cache directory:

```sh
CARGO_BUILD_JOBS=2 cargo test --locked --offline --release --manifest-path absolutejs/regression/Cargo.toml
```

The original tree capture, provenance, baseline failure, traces and 40-case scale/depth sweep are committed to `alexkahndev/hello-world-absolutejs`, `docs/physics-build-2026-10-27/`. Across 360 normalized single-precision query controls per build, maximum distance error against the analytic floor plane falls from 14.720223 cm to 0.000695405 cm. These isolated results do not qualify a new game solver. WASM snapshot replay and full tree-contact tests remain required in `@absolutejs/rapier3d` before adopting or publishing its native change.

Consumers must pin a tested source revision. This fork is not a crates.io release and does not alter the upstream crate version. The original Apache-2.0 license and authorship remain intact.
