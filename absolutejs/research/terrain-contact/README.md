# Prior-direction terrain contact regression

Base: published Parry 8c68a62, shipped in rapier3d 0.17.0. Standalone fixture
comes from native onset reduction at engine 18bb4a9: step 877 recycle-base
pose, triangle 999, actual convex topology and previous cached direction.
The expected near contact is about 0.01 cm apart within a 2 cm prediction
distance. Original code returns an empty manifold with normal X. The baseline
assertion fails at 1374db4. Fresh and recentered controls find the contact.

Locked dependencies exactly match the previously qualified rotational fixture
manifest. A temporary std-only dimension trace is under investigation; it is
not a production change or a new public release. Canonical game remains 0.17.0.
