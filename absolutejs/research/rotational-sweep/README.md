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
