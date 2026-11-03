use parry3d::{
    mass_properties::MassProperties,
    math::{Real, Vector},
    shape::{ConvexPolyhedron, Shape},
};
#[cfg(feature = "f64")]
use parry3d_f64 as parry3d;

fn ridge(height: Real, scale: Real) -> (Vec<Vector>, Vec<[u32; 3]>) {
    let points = [
        [-1., -1., -1.],
        [1., -1., -1.],
        [1., -1., 1.],
        [-1., -1., 1.],
        [-1., 1., -1.],
        [1., 1., -1.],
        [1., 1., 1.],
        [-1., 1., 1.],
        [0., 1. + height, 0.],
    ];
    let indices = vec![
        [0, 1, 2],
        [0, 2, 3],
        [0, 4, 5],
        [0, 5, 1],
        [1, 5, 6],
        [1, 6, 2],
        [3, 2, 6],
        [3, 6, 7],
        [0, 3, 7],
        [0, 7, 4],
        [4, 8, 5],
        [5, 8, 6],
        [6, 8, 7],
        [7, 8, 4],
    ];
    (
        points
            .into_iter()
            .map(|p| Vector::new(p[0] * scale, p[1] * scale, p[2] * scale))
            .collect(),
        indices,
    )
}
#[test]
fn shallow_convex_ridge_preserves_volume_instead_of_becoming_a_flat_face() {
    for scale in [0.001, 1.0, 100.0, 10000.0] {
        let height: Real = if cfg!(feature = "f64") { 1e-5 } else { 0.01 };
        let (points, indices) = ridge(height, scale);
        let shape = ConvexPolyhedron::from_convex_mesh(points, &indices).unwrap();
        let expected = (8.0 + 4.0 * height / 3.0) * scale.powi(3);
        let actual = shape.mass_properties(1.0).mass();
        let relative = (actual - expected).abs() / expected;
        println!(
            "scale={scale} expected={expected} actual={actual} faces={} relative={relative}",
            shape.faces().len()
        );
        assert!(
            relative < if cfg!(feature = "f64") { 1e-12 } else { 2e-6 },
            "shallow convex ridge was flattened: {relative}"
        );
    }
}
#[test]
fn genuinely_coplanar_cube_faces_still_merge() {
    let (points, indices) = ridge(0., 1.);
    let shape = ConvexPolyhedron::from_convex_mesh(points, &indices).unwrap();
    assert_eq!(shape.faces().len(), 6);
    assert!((shape.mass_properties(1.).mass() - 8.).abs() < 1e-6);
}

#[test]
fn affine_coplanar_faces_still_merge() {
    let (mut points, indices) = ridge(0., 1.);
    for p in &mut points {
        p.x += p.y * 0.25;
        p.z += p.x * 0.5;
        *p += Vector::new(1024., -2048., 4096.);
    }
    let expected = MassProperties::from_trimesh(1., &points, &indices).mass();
    let shape = ConvexPolyhedron::from_convex_mesh(points, &indices).unwrap();
    assert_eq!(shape.faces().len(), 6);
    assert!((shape.mass_properties(1.).mass() - expected).abs() / expected < 2e-6);
}
#[test]
fn captured_stone_preserves_input_triangle_mass() {
    let points: Vec<Vector> = STONE_POINTS
        .iter()
        .map(|p| Vector::new(p[0] as Real, p[1] as Real, p[2] as Real))
        .collect();
    let expected = MassProperties::from_trimesh(1., &points, &STONE_INDICES).mass();
    let shape = ConvexPolyhedron::from_convex_mesh(points, &STONE_INDICES).unwrap();
    let actual = shape.mass_properties(1.).mass();
    let error = (actual - expected).abs() / expected;
    println!(
        "captured stone: faces={} relative mass error={error}",
        shape.faces().len()
    );
    assert!(error < if cfg!(feature = "f64") { 1e-12 } else { 2e-6 });
}
include!("fixtures/convex_stone.rs");
