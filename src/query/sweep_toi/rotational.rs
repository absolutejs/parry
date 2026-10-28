//! Fixed-plane conservative advancement for endpoint-interpolated 3D rotations.
//! Uses double-precision local motion bounds; public geometry remains Real.
use crate::{
    math::{Real, Vector},
    query::sweep_toi::{
        SimplexCache, Sweep, SweepToiOutput, SweepToiStatus, ToiProxy, proxy_distance,
    },
};

use glamx::{DQuat, DVec3};
fn vec(v: Vector) -> DVec3 {
    DVec3::new(v.x as f64, v.y as f64, v.z as f64)
}
fn endpoints(s: &Sweep) -> (DQuat, DQuat) {
    let a = DQuat::from_xyzw(s.q1.x as f64, s.q1.y as f64, s.q1.z as f64, s.q1.w as f64);
    let b = DQuat::from_xyzw(s.q2.x as f64, s.q2.y as f64, s.q2.z as f64, s.q2.w as f64);
    (if a.dot(b) < 0.0 { -a } else { a }, b)
}
fn curvature(s: &Sweep, p: &ToiProxy) -> f64 {
    let (a, b) = endpoints(s);
    let delta = b - a;
    let extremum = if delta.length_squared() > 0.0 {
        (-a.dot(delta) / delta.length_squared()).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let min_norm_sq = (a + delta * extremum).length_squared().next_down();
    let radius = p
        .points()
        .iter()
        .map(|v| (vec(*v) - vec(s.local_center)).length())
        .fold(0.0_f64, f64::max);
    // u=q0+t*d, q=u/|u|: |q'|<=|d|/m, |q''|<=3|d|²/m².
    // R(q)v=q*v*q*: |(R(q)v)''|<=8|v||d|²/m².
    (8.0 * radius * delta.length_squared() / min_norm_sq).next_up()
}
fn projected_pair(
    s: &Sweep,
    p: &ToiProxy,
    t: Real,
    axes: [Vector; 2],
) -> [(f64, f64, f64, f64, f64); 2] {
    let (a, b) = endpoints(s);
    let d = b - a;
    let u = a + d * (t as f64);
    let norm = u.length();
    let q = u / norm;
    let dq = (d - q * q.dot(d)) / norm;
    let w = dq * q.conjugate();
    let omega = DVec3::new(w.x, w.y, w.z) * 2.0;
    let linear = vec(s.c2) - vec(s.c1);
    let center = vec(s.c1) + linear * (t as f64);
    let axes = axes.map(|n| vec(n).normalize_or_zero());
    let mut values = [(f64::MAX, -f64::MAX, f64::MAX, -f64::MAX, 0.0_f64); 2];
    for point in p.points() {
        let arm = q * (vec(*point) - vec(s.local_center));
        let world = center + arm;
        let velocity = linear + omega.cross(arm);
        for (v, axis) in values.iter_mut().zip(axes) {
            let projection = world.dot(axis);
            let speed = velocity.dot(axis);
            v.0 = v.0.min(projection);
            v.1 = v.1.max(projection);
            v.2 = v.2.min(speed);
            v.3 = v.3.max(speed);
            v.4 = v.4.max(world.abs().dot(axis.abs()));
        }
    }
    values
}
pub(super) fn rotational_time_of_impact(
    a: &ToiProxy,
    sa: &Sweep,
    b: &ToiProxy,
    sb: &Sweep,
    max_fraction: Real,
    slop: Real,
) -> SweepToiOutput {
    let origin = sa.c1;
    let sa = sa.shifted(origin);
    let sb = sb.shifted(origin);
    let target = slop.max(a.radius + b.radius - slop);
    let tolerance = 0.25 * slop;
    let bound = curvature(&sa, a) + curvature(&sb, b);
    let mut cache = SimplexCache::default();
    let mut t = 0.0;
    let mut out = SweepToiOutput {
        status: SweepToiStatus::Failed,
        fraction: 0.0,
        point: Vector::ZERO,
        normal: Vector::ZERO,
    };
    for _ in 0..25 {
        let xa = sa.transform_at(t);
        let xb = sb.transform_at(t);
        let distance = proxy_distance(&xa.inv_mul(&xb), a, b, false, &mut cache);
        let pa = xa.transform_point(distance.point_a);
        let pb = xa.transform_point(distance.point_b);
        let witness_n = (pb - pa).normalize_or_zero();
        let gjk_n = xa.rotation * distance.normal;
        let axes = [witness_n, gjk_n];
        let ap = projected_pair(&sa, a, t, axes);
        let bp = projected_pair(&sb, b, t, axes);
        let selected = if bp[1].0 - ap[1].1 > bp[0].0 - ap[0].1 {
            1
        } else {
            0
        };
        let n = axes[selected];
        out.fraction = t;
        out.point = (pa + pb + (a.radius - b.radius) * n) * 0.5 + origin;
        out.normal = n;
        if distance.distance <= 0.0 {
            out.status = if t == 0.0 {
                SweepToiStatus::Overlapped
            } else {
                SweepToiStatus::Failed
            };
            return out;
        }
        if distance.distance <= target + tolerance {
            out.status = SweepToiStatus::Hit;
            return out;
        }
        if t == max_fraction {
            out.status = SweepToiStatus::Separated;
            return out;
        }
        if n == Vector::ZERO {
            return out;
        }
        let (_, amax, _, avmax, amag) = ap[selected];
        let (bmin, _, bvmin, _, bmag) = bp[selected];
        let roundoff = 64.0 * f64::EPSILON * (amag + bmag + target as f64);
        let margin = bmin - amax - target as f64 - roundoff;
        if margin <= 0.0 {
            return out;
        }
        let closing = (avmax - bvmin).max(0.0);
        let denom = closing + (closing * closing + 2.0 * bound * margin).sqrt();
        let safe = if denom > 0.0 {
            2.0 * margin / denom
        } else {
            f64::INFINITY
        };
        if safe > (max_fraction - t) as f64 {
            out.status = SweepToiStatus::Separated;
            out.fraction = max_fraction;
            return out;
        }
        let next = ((t as f64 + 0.9 * safe).min(max_fraction as f64) as Real).next_down();
        if !next.is_finite() || next <= t {
            return out;
        }
        t = next;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        math::{Pose, Rotation},
        shape::{Cuboid, Triangle},
    };

    #[test]
    fn rotating_clear_endpoints_find_first_plane_impact() {
        for scale in [0.01, 1.0, 100.0] {
            let triangle = Triangle::new(
                Vector::new(-10.0, 0.0, -10.0) * scale,
                Vector::new(0.0, 0.0, 10.0) * scale,
                Vector::new(10.0, 0.0, -10.0) * scale,
            );
            let bar = Cuboid::new(Vector::new(2.0, 0.001, 0.01) * scale);
            let a = ToiProxy::from_shape(&triangle).unwrap();
            let b = ToiProxy::from_shape(&bar).unwrap();
            let angle = |degrees: Real| degrees * (core::f64::consts::PI as Real) / 180.0;
            let moving = Sweep::from_poses(
                &Pose::from_parts(
                    Vector::new(0.0, 1.999, 0.0) * scale,
                    Rotation::from_rotation_z(angle(87.0)),
                ),
                &Pose::from_parts(
                    Vector::new(0.0, 1.999, 0.0) * scale,
                    Rotation::from_rotation_z(angle(93.0)),
                ),
                Vector::ZERO,
            );
            let floor = Sweep::constant(&Pose::IDENTITY, Vector::ZERO);
            for reverse in [false, true] {
                let call = |clip| {
                    if reverse {
                        crate::query::sweep_toi::sweep_time_of_impact(
                            &b,
                            &moving,
                            &a,
                            &floor,
                            clip,
                            0.00001 * scale,
                        )
                    } else {
                        crate::query::sweep_toi::sweep_time_of_impact(
                            &a,
                            &floor,
                            &b,
                            &moving,
                            clip,
                            0.00001 * scale,
                        )
                    }
                };
                let clear = call(0.1);
                assert_eq!(clear.status, SweepToiStatus::Separated);
                let hit = call(1.0);
                assert_eq!(
                    hit.status,
                    SweepToiStatus::Hit,
                    "scale {scale}, reverse {reverse}: {hit:?}"
                );
                assert!(hit.fraction > 0.1 && hit.fraction < 0.5);
                let gap = b
                    .points()
                    .iter()
                    .map(|p| moving.transform_at(hit.fraction).transform_point(*p).y / scale)
                    .fold(Real::MAX, Real::min);
                assert!(
                    gap >= 0.00001 - 8.0 * Real::EPSILON && gap <= 0.0000125 + 8.0 * Real::EPSILON,
                    "gap {gap}"
                );
            }
        }
    }

    #[test]
    fn projected_acceleration_bound_covers_sampled_offset_sweeps() {
        let points = [
            Vector::new(-2.0, 0.03, 0.2),
            Vector::new(1.0, -0.1, -0.4),
            Vector::new(0.1, 0.8, 0.3),
        ];
        let proxy = ToiProxy::from_points(&points, 0.0);
        for angle in [0.01, 0.5, 1.5, 3.0] {
            for center in [Vector::ZERO, Vector::new(0.7, -0.4, 0.2)] {
                let a = Sweep::from_poses(
                    &Pose::translation(-0.3, 0.2, 0.1),
                    &Pose::from_parts(
                        Vector::new(0.4, -0.3, 0.2),
                        Rotation::from_rotation_z(angle),
                    ),
                    center,
                );
                let b = Sweep::from_poses(
                    &Pose::translation(0.2, 0.7, -0.3),
                    &Pose::from_parts(
                        Vector::new(-0.5, 0.4, 0.1),
                        Rotation::from_rotation_x(-angle),
                    ),
                    -center,
                );
                let bound = curvature(&a, &proxy) + curvature(&b, &proxy);
                for axis in [
                    Vector::X,
                    Vector::Y,
                    Vector::Z,
                    Vector::new(1.0, 2.0, -1.0).normalize(),
                ] {
                    for t in [0.0, 0.1, 0.5, 0.8] {
                        let aa = projected_pair(&a, &proxy, t, [axis; 2])[0];
                        let bb = projected_pair(&b, &proxy, t, [axis; 2])[0];
                        for fraction in [0.0, 0.1, 0.5, 1.0] {
                            let end = t + (1.0 - t) * fraction;
                            let h = end as f64 - t as f64;
                            let lower = bb.0 - aa.1 + (bb.2 - aa.3) * h - 0.5 * bound * h * h;
                            let later_a = projected_pair(&a, &proxy, end, [axis; 2])[0];
                            let later_b = projected_pair(&b, &proxy, end, [axis; 2])[0];
                            assert!(
                                later_b.0 - later_a.1 >= lower - 1e-10,
                                "angle {angle}, t {t}, end {end}"
                            );
                        }
                    }
                }
            }
        }
    }
}
