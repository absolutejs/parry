# Certified GJK normal investigation (private)

Base: published Parry c566aabd77be8ac7a427c2b704860c47e34657b9.
This branch does not contain the rejected persistent heightfield-frame change.

The exact game fixtures and four forward/reverse, before/after cached-query
controls are copied from the preceding reduction. On unchanged source, the
before/heightfield-first case reports -31.968266 centimetres while the other
three pass (baseline evidence in research/triangle-contact-conditioning).
The independent f64 plane-separation calculation bounds actual necessary
translation below 0.000005 millimetres for that before-state geometry.

## Hypothesis

GJK stores the strongest observed support lower bound to decide whether a
stalled/duplicate simplex is separated or touching. It can then return a normal
from a different, later search direction. The classification therefore need
not certify the direction used by polygon clipping. Track the direction paired
with that bound and use it for these stalled returns. Leave ordinary convergence,
intersection/EPA handling, tolerances, support points and contact frames unchanged.
The candidate preserves witness points; verify that they and the selected
normal remain geometrically coherent, rather than relying only on test counts.

This is a hypothesis under qualification. A focused pass is not sufficient:
run broader collision cases, package tests, real-tree comparisons, snapshot
replay and independent builds before any release. Do not promote the candidate
by increasing the game clearance/contact limits or solver iteration budget.

Use one compiler job, durable evidence under ~/alex and no native GPU tests.

The four exact fixture controls now pass without translating heightfield contact
frames. Full library testing is underway. The upstream workspace first refused
offline resolution of unrelated optional `encase` dependencies; selecting the
non-member library from the small harness is also unsupported by Cargo. A
durable generated test-only manifest therefore uses the exact library source,
the selected runtime feature/dependency closure, and `oorandom` (the only dev
dependency used by source unit tests). It does not compile GUI examples or
change production dependencies. Source hashes and the exact manifest are saved.

This exposed pre-existing compile errors in two witness regression tests:
missing `alloc::vec::Vec` and six intentionally discarded must-use projections.
The test module now imports Vec and explicitly discards those return values;
its assertions and runtime library logic are unchanged. Failed compile logs
are retained. The log named `library-tests-import-corrected.log` still failed
because the first edit targeted the wrong module name; it is not passing evidence.
