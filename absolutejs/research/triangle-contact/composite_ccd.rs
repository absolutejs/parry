use parry3d::{
    math::{Pose, Rotation, Vector},
    query::sweep_toi::{
        sweep_time_of_impact, sweep_time_of_impact_composite, Sweep, SweepCompositeFastShape,
        SweepToiStatus, ToiProxy,
    },
    shape::{Cuboid, HeightField, Shape, TriMesh, Triangle},
    utils::Array2,
};
#[test]
fn rotating_tip_crosses_one_sided_heightfield_with_stationary_centroid() {
    let floor = HeightField::new(Array2::repeat(3, 3, 0.0), Vector::new(10.0, 1.0, 10.0));
    let bar = Cuboid::new(Vector::new(2.0, 0.01, 0.1));
    let proxy = ToiProxy::from_shape(&bar).unwrap();
    let start = Pose::translation(0.0, 1.0, 0.0);
    let end = Pose::from_parts(
        start.translation,
        Rotation::from_rotation_z(40.0_f32.to_radians()),
    );
    // Analytic extremal corner: initially above, finally below the horizontal plane.
    assert!(1.0 - bar.half_extents.y > 0.0);
    assert!(1.0 - 2.0 * 40.0_f32.to_radians().sin() - 0.01 * 40.0_f32.to_radians().cos() < 0.0);
    let sweep = Sweep::from_poses(&start, &end, Vector::ZERO);
    let fast = SweepCompositeFastShape {
        proxy: &proxy,
        sweep: &sweep,
        local_centroid: Vector::ZERO,
        min_extent: bar.ccd_thickness(),
    };
    let two_sided =
        sweep_time_of_impact_composite(&floor, &Pose::IDENTITY, fast, false, false, 1.0, 0.0001)
            .unwrap();
    assert_eq!(
        two_sided.status,
        SweepToiStatus::Hit,
        "control {two_sided:?}"
    );
    let one_sided =
        sweep_time_of_impact_composite(&floor, &Pose::IDENTITY, fast, true, false, 1.0, 0.0001)
            .unwrap();
    assert_eq!(
        one_sided.status,
        SweepToiStatus::Hit,
        "one-sided {one_sided:?}; two-sided {two_sided:?}"
    );
    assert!(one_sided.fraction > 0.0 && one_sided.fraction < 1.0);
}
#[test]
fn composite_sweep_bounds_include_rotational_arc_between_clear_endpoints() {
    let triangle = Triangle::new(
        Vector::new(-10.0, 0.0, -10.0),
        Vector::new(0.0, 0.0, 10.0),
        Vector::new(10.0, 0.0, -10.0),
    );
    let mesh = TriMesh::new(vec![triangle.a, triangle.b, triangle.c], vec![[0, 1, 2]]).unwrap();
    let bar = Cuboid::new(Vector::new(2.0, 0.001, 0.01));
    let proxy = ToiProxy::from_shape(&bar).unwrap();
    let start = Pose::from_parts(
        Vector::new(0.0, 1.999, 0.0),
        Rotation::from_rotation_z(87.0_f32.to_radians()),
    );
    let end = Pose::from_parts(
        start.translation,
        Rotation::from_rotation_z(93.0_f32.to_radians()),
    );
    let sweep = Sweep::from_poses(&start, &end, Vector::ZERO);
    assert!(bar.compute_aabb(&start).mins.y > 0.0);
    assert!(bar.compute_aabb(&end).mins.y > 0.0);
    assert!(bar.compute_aabb(&sweep.transform_at(0.5)).mins.y < 0.0);
    let triangle_proxy = ToiProxy::from_shape(&triangle).unwrap();
    let direct = sweep_time_of_impact(
        &triangle_proxy,
        &Sweep::constant(&Pose::IDENTITY, Vector::ZERO),
        &proxy,
        &sweep,
        1.0,
        0.00001,
    );
    assert_eq!(direct.status, SweepToiStatus::Hit, "direct {direct:?}");
    let fast = SweepCompositeFastShape {
        proxy: &proxy,
        sweep: &sweep,
        local_centroid: Vector::ZERO,
        min_extent: bar.ccd_thickness(),
    };
    let composite =
        sweep_time_of_impact_composite(&mesh, &Pose::IDENTITY, fast, false, false, 1.0, 0.00001)
            .unwrap();
    assert_eq!(
        composite.status,
        SweepToiStatus::Hit,
        "composite {composite:?}; direct {direct:?}"
    );
}
