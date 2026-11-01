# Captured Float32 hull boundary investigation

Baseline Parry commit53ac0b0b02767300da09d092d97cef53a975c947, the unchanged dependency of qualified Rapier0.20.0. Native host source is separate from all qualified archives and game production pins. Builds are offline and single-job; no GPU or publication.

The39-point input is the original Float32 point cloud of game variant4/broad/stage0 collider1. Source capture and exact public-WASM replay are backed up in the game research/fluid-film-controls branch at64afada9, with the export-vs-internal correction at2b48cb00. Input words are preserved as integers in src/input.rs.

The host initial Quickhull, stored-polyhedron fan triangulation and second Quickhull used by the public Rapier export all contain strict exact noncoplanar triangle crossings. Public export positions/indices match WASM exactly. The initial and exported boundaries differ in indexing, so they are retained separately. A plane violation is not itself displacement error, and these exact checks are not epsilon-validity certificates.

The double implementation receives exactly the same Float32 values promoted losslessly. It retains39vertices74triangles; exact rational checks find no strict crossing or outward halfspace violation. No precision-change package is yet implemented or approved by this result.

The bounded reducer preserves an exact strict public-export crossing and removes points greedily. It finishes160evaluations in4.75sec with12points; a complete single-point deletion sweep cannot reduce that further. This is not globally minimal and does not claim to preserve the original negative Boolean. All accepted meshes, original retained indexes, input words and attempt outcomes are durable in reduction/. Rejected attempts are not substituted for physical outputs.

Commands: CARGO_HOME=~/alex/rapier-simplex-gyro-release/.cache/cargo CARGO_NET_OFFLINE=true CARGO_BUILD_JOBS=1 cargo run --release --manifest-path absolutejs/research/captured-hull/Cargo.toml. Optional first binary argument is a whitespace-separated u32 Float32-bit input file. Python analyzer uses exact fractions for strict crossing and halfspace predicates. Each evidence output refuses overwrite. Cargo lock is explicitly tracked for the isolated research crate; generated target files remain ignored.

The first baseline build was still compiling dependency code when the probe main was extended to include the second export stage. It is exploratory evidence, not a frozen-input qualification. `baseline-main.rs.txt` preserves the resulting probe source; subsequent release qualification must freeze all source and compare independent builds.
