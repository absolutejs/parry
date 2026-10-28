#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::{Pose, Rotation};

    fn slop() -> Real {
        0.005
    }

    #[cfg(feature = "dim2")]
    fn pose(x: Real, y: Real) -> Pose {
        Pose::from_parts(Vector::new(x, y), Rotation::identity())
    }
    #[cfg(feature = "dim3")]
    fn pose(x: Real, y: Real) -> Pose {
        Pose::from_parts(Vector::new(x, y, 0.0), Rotation::IDENTITY)
    }

    fn ball_proxy(radius: Real) -> ToiProxy<'static> {
        ToiProxy::point(Vector::ZERO, radius)
    }

    fn cuboid_proxy(half_extent: Real) -> ToiProxy<'static> {
        #[cfg(feature = "dim2")]
        {
            ToiProxy::from_array(
                [
                    Vector::new(-half_extent, -half_extent),
                    Vector::new(half_extent, -half_extent),
                    Vector::new(half_extent, half_extent),
                    Vector::new(-half_extent, half_extent),
                ],
                0.0,
            )
        }
        #[cfg(feature = "dim3")]
        {
            let h = half_extent;
            ToiProxy::from_array(
                [
                    Vector::new(-h, -h, -h),
                    Vector::new(h, -h, -h),
                    Vector::new(h, h, -h),
                    Vector::new(-h, h, -h),
                    Vector::new(-h, -h, h),
                    Vector::new(h, -h, h),
                    Vector::new(h, h, h),
                    Vector::new(-h, h, h),
                ],
                0.0,
            )
        }
    }

    #[test]
    fn ball_hits_static_box() {
        // Static unit box at the origin; ball of radius 0.1 sweeping from x = -3 to x = +3.
        let wall = cuboid_proxy(0.5);
        let wall_sweep = Sweep::constant(&pose(0.0, 0.0), Vector::ZERO);
        let ball = ball_proxy(0.1);
        let ball_sweep = Sweep::from_poses(&pose(-3.0, 0.0), &pose(3.0, 0.0), Vector::ZERO);

        let result = sweep_time_of_impact(&wall, &wall_sweep, &ball, &ball_sweep, 1.0, slop());
        assert_eq!(result.status, SweepToiStatus::Hit);

        // The ball is stopped when its core (center) is `target = radius - slop` away from
        // the box surface: center_x = -0.5 - (0.1 - 0.005), fraction = (start - x) / travel.
        let expected = (3.0 - 0.5 - (0.1 - slop())) / 6.0;
        assert!(
            (result.fraction - expected).abs() < 0.001,
            "fraction {} vs expected {expected}",
            result.fraction
        );
    }

    #[test]
    fn ball_misses_box() {
        // Ball sweeping parallel to the box, far away.
        let wall = cuboid_proxy(0.5);
        let wall_sweep = Sweep::constant(&pose(0.0, 0.0), Vector::ZERO);
        let ball = ball_proxy(0.1);
        let ball_sweep = Sweep::from_poses(&pose(-3.0, 5.0), &pose(3.0, 5.0), Vector::ZERO);

        let result = sweep_time_of_impact(&wall, &wall_sweep, &ball, &ball_sweep, 1.0, slop());
        assert_eq!(result.status, SweepToiStatus::Separated);
        assert_eq!(result.fraction, 1.0);
    }

    #[test]
    fn overlapped_at_start_returns_fraction_zero() {
        let wall = cuboid_proxy(0.5);
        let wall_sweep = Sweep::constant(&pose(0.0, 0.0), Vector::ZERO);
        let ball = ball_proxy(0.1);
        let ball_sweep = Sweep::from_poses(&pose(0.0, 0.0), &pose(3.0, 0.0), Vector::ZERO);

        let result = sweep_time_of_impact(&wall, &wall_sweep, &ball, &ball_sweep, 1.0, slop());
        assert_eq!(result.status, SweepToiStatus::Overlapped);
        assert_eq!(result.fraction, 0.0);
    }

    #[test]
    fn box_vs_box_face_impact() {
        // Two unit boxes; one sweeps into the other along x.
        let a = cuboid_proxy(0.5);
        let a_sweep = Sweep::constant(&pose(0.0, 0.0), Vector::ZERO);
        let b = cuboid_proxy(0.5);
        let b_sweep = Sweep::from_poses(&pose(-4.0, 0.0), &pose(4.0, 0.0), Vector::ZERO);

        let result = sweep_time_of_impact(&a, &a_sweep, &b, &b_sweep, 1.0, slop());
        assert_eq!(result.status, SweepToiStatus::Hit);

        // Faces meet when the gap reaches target = slop: center_x = -(0.5 + 0.5 + slop).
        let expected = (4.0 - 1.0 - slop()) / 8.0;
        assert!(
            (result.fraction - expected).abs() < 0.001,
            "fraction {} vs expected {expected}",
            result.fraction
        );
    }

    #[test]
    fn rotating_bar_hits_ball() {
        // A long thin bar rotating 90° about the origin must catch a ball placed along its
        // swept arc even though the bar’s endpoint poses don’t overlap the ball.
        let half_length = 2.0;
        #[cfg(feature = "dim2")]
        let (bar, start, end) = (
            ToiProxy::from_array(
                [
                    Vector::new(-half_length, 0.0),
                    Vector::new(half_length, 0.0),
                ],
                0.05,
            ),
            Pose::from_parts(Vector::ZERO, Rotation::identity()),
            Pose::from_parts(
                Vector::ZERO,
                Rotation::from_angle(core::f32::consts::FRAC_PI_2 as Real),
            ),
        );
        #[cfg(feature = "dim3")]
        let (bar, start, end) = (
            ToiProxy::from_array(
                [
                    Vector::new(-half_length, 0.0, 0.0),
                    Vector::new(half_length, 0.0, 0.0),
                ],
                0.05,
            ),
            Pose::from_parts(Vector::ZERO, Rotation::IDENTITY),
            Pose::from_parts(
                Vector::ZERO,
                Rotation::from_rotation_z(core::f32::consts::FRAC_PI_2 as Real),
            ),
        );

        let bar_sweep = Sweep::from_poses(&start, &end, Vector::ZERO);
        let ball = ball_proxy(0.1);
        // Place the ball at 45° on the arc of radius 1.5.
        let d = 1.5 * (0.5_f64.sqrt() as Real);
        let ball_sweep = Sweep::constant(&pose(d, d), Vector::ZERO);

        let result = sweep_time_of_impact(&bar, &bar_sweep, &ball, &ball_sweep, 1.0, slop());
        assert_eq!(result.status, SweepToiStatus::Hit);
        // The bar reaches 45° at t = 0.5; it should hit slightly before.
        assert!(
            result.fraction > 0.3 && result.fraction < 0.5,
            "fraction {}",
            result.fraction
        );
    }
}
