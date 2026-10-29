# Rotational sweep interval investigation

Base is published Parry c566aabd, independent of the blocked triangle-frame,
GJK-normal and composite-CCD candidates. Scope is the direct convex sweep.
A thin box rotates from 87 to 93 degrees about Z, with its center 1.999 above
a horizontal triangle and half-length 2. Both endpoints are clear; its midpoint
crosses the plane. Endpoint separation alone cannot certify the swept interval.

The baseline probe covers three scales, both shape orders and clipped/full
intervals. It reports expected versus observed status without disguising failed
baseline behavior as a passing regression. No source correction or release is
claimed. The existing game penetration issue is not yet proven to come from this
miss; keep its paired replay and acceptance gates separate.

Candidate direction: certify each skipped interval against a fixed separating
plane using relative translation and an angular displacement bound about each
center of mass. Normalized quaternion interpolation is not constant angular
speed, and offset centers must be included. Merely sampling more poses cannot
certify the interval. Preserve bounded query work and explicit failure reporting;
then test earliest impact, grazing/no-hit controls, shape-order symmetry, scales,
clipped intervals, offsets and query cost before any game qualification.

Baseline completes on published source: all six full-interval scale/order cases
report Separated despite an analytically intersecting midpoint. All six clipped
clear controls report Separated as expected. Complete numeric inputs/results are
in baseline.json; compilation used one Cargo job and no GPU.

The first private interval candidate (not library code) replaces endpoint
separation inference with a quadratic bound on fixed-plane separation. It uses
normalized quaternion derivative/curvature and each proxy's radius about its
center of mass, with a 25-query cap. Its initial witness-derived plane is too
weak: all 12 cases return Failed, including clear controls. This is explicitly
rejected, not a fix. Exact source and result are retained before refinement.

The f32 two-plane variant clears all six clipped controls but returns Failed
before all six impacts because its conservative rounding allowance exceeds the
tiny 1e-5 slop. Retained source/results are interval_candidate_f32.rs and
interval-candidate-f32-margin.json. A missing f64 feature compile failure is also
retained. The higher-precision candidate leaves query geometry/API unchanged,
uses f64 only for projected motion bounds, and does not normalize supplied
endpoint quaternions before interpolation. It evaluates the actual affine
segment's minimum norm, including slightly nonunit stored endpoints.

The expanded 432-case plane matrix matches all oracle classifications: 168 Hit,
264 Separated, no Failed; at most 15 distance queries. Scale, shape order, world
orientation, COM offsets, quaternion sign, three rotation spans and clipped time
are covered. The independent oracle explicitly evaluates corner/plane geometry;
its sampling only brackets the test root and is not used by the candidate. These
are initial results, not production engine/game qualification or a general proof.

Strict matrix assertions pass: classification, earliest target crossing, bounded
plane gap, <=25 queries, and exact quaternion-sign invariance. Five unchanged
upstream sweep controls also pass through a wrapper targeting the private
candidate. Cost sampling exposes unacceptable unnecessary work for pure
translation: about 11.25x for the hit control and 5.35x for the miss. Rotating
bar/ball is about 1.68x; rotating clear is 1.91x. The previously missed interior
crossing is about 8.90x, comparing different outcomes. These are native shared-host
measurements, not FPS. Preserve the working translation path and reduce redundant
projection transforms before considering engine integration.

## Native library research integration

The optimized prototype still passes all strict cases; bounded native ratios
are retained in cost-summary.json. Identical translation routing has noisy
measured ratios and does not establish a speedup. The clear rotational query
costs about 1.68x; rotating bar/ball about 0.93x. Previously missed interior
crossing costs about 6.02x while changing Separated to Hit.

The same candidate is now integrated privately into the actual 3D library, with
the unchanged translation route and unchanged 2D code. All 432 native results
match the prototype's status and fraction exactly, and five existing controls
pass. Two actual library unit regressions cover first impact and sampled
projected-acceleration bounds. 3D/f32, 3D/f64 and 2D feature matrices are pending.
This is not a released engine and is not yet game-qualified.

The cost.rs pre-integration benchmark must be run at its recorded source commit
d0b5593 to compare the original engine; after native routing both arms would
call the new library for rotations. Old comparison outputs are retained.

## Native feature qualification

All selected runtime-feature library suites pass: 3D/f32 85, 3D/f64 84,
2D/f32 98 and 2D/f64 98. Allocation-enabled no-std passes for both 3D types.
The published baseline's witness-test module lacked a Vec import and discarded
six return values forbidden by deny(unused_results); test-only fixes preserve
all assertions. Initial logs are retained. The new no-std module needed a scoped
Float import for sqrt; its initial failure and successful retries are retained.
No production GJK algorithm changes are included. Whole-engine/browser/game
qualification and independent builds remain outstanding.

## Combined direction/rotation research

This branch adds only the GJK bound-associated direction correction from
`a26d29f` to native-qualified interval sweep source `627113c`. No frame change
or composite sweep bounds are included. The captured oak fixture fails with
-20.193651 cm on unmodified interval code (`oak-baseline.log`, ab81105). With
the correction it stays within -0.000571 cm in 12 origin/order/repeated-query
controls. The independent captured plane bound is about 0.001265 cm. This
assertion is a fixture bound; no runtime tolerances change.

`combined-matrix.json` is exactly identical to the prior native 432-case report.
All five original sweep controls and four prior captured heightfield controls
pass. Selected library feature sets `required-features,std,alloc` pass
84/83/97/97 tests (3D-f32/3D-f64/2D-f32/2D-f64). Unlike the earlier runs these
sets omit serde, explaining one fewer serialization test in each set. Both 3D
alloc/no-std checks pass. Full engine qualification is still required; prior
GJK-only engines had whole-tree regressions and these checks do not waive them.
