# Not qualification evidence

These commands reused a target directory from another worktree. The library
reported 84/83 tests and did not include the newly added duplicate-CSO unit
test despite that test being present in source. Treat these cached results
as invalid for this candidate. All gates are rerun with a fresh target owned
by this source checkout, and assert the new unit test actually ran.
