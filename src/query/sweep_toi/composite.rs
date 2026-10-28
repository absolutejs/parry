//! Sweep time-of-impact against composite shapes (meshes, polylines, heightfields,
//! compounds): the composite is assumed stationary, its acceleration structure is queried
//! with the swept bounds of the moving shape, and each candidate element runs the convex
//! TOI with a fallback sphere when the element reports an initial overlap.

use super::sweep::Sweep;
use super::sweep_toi::{sweep_time_of_impact, SweepToiOutput, SweepToiStatus};
use super::toi_proxy::ToiProxy;
#[cfg(feature = "dim3")]
use crate::bounding_volume::Aabb;
use crate::bounding_volume::BoundingVolume;
use crate::math::{Pose, Real, Vector};
use crate::shape::{Shape, TypedShape};

#[cfg(feature = "dim2")]
use crate::shape::{PolylineFlags, Segment};
#[cfg(feature = "dim3")]
use crate::shape::{TriMeshFlags, Triangle};

/// Fraction of the fast shape’s minimum extent used for the initial-overlap fallback sphere.
pub const CORE_FRACTION: Real = 0.25;

/// Parameters describing the fast (moving) shape for a composite TOI query.
#[derive(Copy, Clone)]
pub struct SweepCompositeFastShape<'a, 'b> {
    /// Point-cloud proxy of the moving shape.
    pub proxy: &'a ToiProxy<'b>,
    /// Sweep of the moving shape.
    pub sweep: &'a Sweep,
    /// Centroid of the moving shape in its local frame.
    pub local_centroid: Vector,
    /// Smallest extent (inner radius) of the moving shape, used for fallback spheres and
    /// one-sided early-outs.
    pub min_extent: Real,
}

struct CompositeToiContext<'a, 'b> {
    fast: SweepCompositeFastShape<'a, 'b>,
    // Centroid of the moving shape at the sweep endpoints, in the composite’s local frame.
    local_centroid1: Vector,
    local_centroid2: Vector,
    fallback_radius: Real,
    one_sided: bool,
    #[cfg_attr(feature = "dim2", allow(dead_code))]
    target_is_sensor: bool,
    #[cfg(feature = "dim3")]
    swept_bounds: Aabb,
    linear_slop: Real,
    max_fraction: Real,
    best: Option<SweepToiOutput>,
}

impl CompositeToiContext<'_, '_> {
    /// Runs the convex TOI of the moving shape against one composite element and keeps the
    /// earliest hit, with a fallback-sphere retry on initial overlap.
    fn toi_against_element(&mut self, element_proxy: &ToiProxy, element_sweep: &Sweep) {
        let output = sweep_time_of_impact(
            element_proxy,
            element_sweep,
            self.fast.proxy,
            self.fast.sweep,
            self.max_fraction,
            self.linear_slop,
        );

        if 0.0 < output.fraction && output.fraction < self.max_fraction {
            self.max_fraction = output.fraction;
            self.best = Some(output);
        } else if output.fraction == 0.0 {
            // Fallback to the TOI of a small ball around the fast shape centroid.
            #[cfg(feature = "dim2")]
            let radius = self.fallback_radius;
            #[cfg(feature = "dim3")]
            let radius = self.fallback_radius + self.linear_slop;

            let fallback_proxy = ToiProxy::point(self.fast.local_centroid, radius);
            let output = sweep_time_of_impact(
                element_proxy,
                element_sweep,
                &fallback_proxy,
                self.fast.sweep,
                self.max_fraction,
                self.linear_slop,
            );

            if 0.0 < output.fraction && output.fraction < self.max_fraction {
                self.max_fraction = output.fraction;
                self.best = Some(output);
            }
        }
    }

    /// One-sided early-out for a 2D chain segment.
    /// Returns `true` when the element can be skipped.
    #[cfg(feature = "dim2")]
    fn one_sided_early_out(&self, segment: &Segment) -> bool {
        if !self.one_sided {
            return false;
        }

        let e = segment.b - segment.a;
        let length = e.length();
        if length <= self.linear_slop {
            return false;
        }
        let e = e / length;

        let separation1 = (self.local_centroid1 - segment.a).perp_dot(e);
        let separation2 = (self.local_centroid2 - segment.a).perp_dot(e);
        let core_distance = CORE_FRACTION * self.fast.min_extent;

        separation1 < 0.0
            || (separation1 - separation2 < core_distance && separation2 > core_distance)
    }

    /// One-sided early-out for a 3D triangle.
    /// Returns `true` when the element can be skipped.
    #[cfg(feature = "dim3")]
    fn one_sided_early_out(&self, triangle: &Triangle) -> bool {
        if !self.one_sided {
            return false;
        }

        let n = (triangle.b - triangle.a)
            .cross(triangle.c - triangle.a)
            .normalize_or_zero();
        let offset1 = n.dot(self.local_centroid1 - triangle.a);

        if offset1 < 0.0 {
            // Started behind.
            return true;
        }

        // A centroid/minimum-radius test does not bound an elongated shape's
        // rotating tips. Only skip when its entire swept bound stays in front.
        let corner = Vector::new(
            if n.x >= 0.0 {
                self.swept_bounds.mins.x
            } else {
                self.swept_bounds.maxs.x
            },
            if n.y >= 0.0 {
                self.swept_bounds.mins.y
            } else {
                self.swept_bounds.maxs.y
            },
            if n.z >= 0.0 {
                self.swept_bounds.mins.z
            } else {
                self.swept_bounds.maxs.z
            },
        );
        if !self.target_is_sensor && n.dot(corner - triangle.a) > 0.0 {
            return true;
        }

        false
    }
}

/// Bounds the full nlerp arc, not just its endpoint boxes. With unit
/// quaternions, every point's rotational displacement is at most
/// `2 * radius * min(|q1-q2|, |q1+q2|)` along the shortest arc. The COM follows
/// a line, so sweeping the start-oriented box along that line and expanding by
/// this bound encloses the whole motion. Proxy inflation is rotation-invariant.
#[cfg(feature = "dim3")]
fn swept_proxy_aabb(proxy: &ToiProxy, sweep: &Sweep, max_fraction: Real) -> Aabb {
    let start = sweep.transform_at(0.0);
    let end = sweep.transform_at(max_fraction);
    let start_box = proxy.compute_aabb(&start);
    let translation = sweep.c1.lerp(sweep.c2, max_fraction) - sweep.c1;
    let translated_box = Aabb::new(start_box.mins + translation, start_box.maxs + translation);
    let radius = proxy
        .points()
        .iter()
        .map(|point| (*point - sweep.local_center).length())
        .fold(0.0 as Real, Real::max);
    let delta = (start.rotation - end.rotation)
        .length()
        .min((start.rotation + end.rotation).length());
    let angular_bound = if delta == 0.0 {
        0.0
    } else {
        (2.0 * radius * delta).next_up()
    };
    start_box.merged(&translated_box).loosened(angular_bound)
}

/// Computes the time of impact between a stationary composite shape and a moving convex
/// shape.
///
/// Returns `None` if `composite` is not a supported composite shape (triangle mesh,
/// polyline, heightfield, or compound). When supported but nothing is hit, the returned
/// output has status [`SweepToiStatus::Separated`] and `fraction == max_fraction`.
///
/// `one_sided` enables the “started behind / finished in front” early-outs; it
/// should only be set for composites whose elements have meaningful outward normals
/// (oriented polylines, oriented meshes, heightfields).
#[allow(clippy::too_many_arguments)]
pub fn sweep_time_of_impact_composite(
    composite: &dyn Shape,
    composite_pose: &Pose,
    fast: SweepCompositeFastShape,
    one_sided: bool,
    target_is_sensor: bool,
    max_fraction: Real,
    linear_slop: Real,
) -> Option<SweepToiOutput> {
    let typed = composite.as_typed_shape();

    // Fallback sphere radius (2D: core circle; 3D: mesh/compound fallback spheres).
    #[cfg(feature = "dim2")]
    let fallback_radius = CORE_FRACTION * fast.min_extent;
    #[cfg(feature = "dim3")]
    let fallback_radius = match typed {
        TypedShape::Compound(_) => (0.75 * fast.min_extent).max(4.0 * linear_slop),
        _ => (0.5 * fast.min_extent).max(linear_slop),
    };

    // Swept bounds of the fast shape, in the composite’s local frame.
    #[cfg(feature = "dim2")]
    let local_aabb = {
        let start_aabb = fast.proxy.compute_aabb(&fast.sweep.transform_at(0.0));
        let end_aabb = fast
            .proxy
            .compute_aabb(&fast.sweep.transform_at(max_fraction));
        start_aabb
            .merged(&end_aabb)
            .transform_by(&composite_pose.inverse())
    };
    #[cfg(feature = "dim3")]
    let local_aabb = swept_proxy_aabb(fast.proxy, fast.sweep, max_fraction)
        .transform_by(&composite_pose.inverse())
        .loosened(linear_slop);

    // Centroid of the fast shape at the sweep endpoints, in the composite’s local frame.
    let centroid_world1 = fast
        .sweep
        .transform_at(0.0)
        .transform_point(fast.local_centroid);
    let centroid_world2 = fast
        .sweep
        .final_transform()
        .transform_point(fast.local_centroid);

    let mut context = CompositeToiContext {
        fast,
        local_centroid1: composite_pose.inverse_transform_point(centroid_world1),
        local_centroid2: composite_pose.inverse_transform_point(centroid_world2),
        fallback_radius,
        one_sided,
        target_is_sensor,
        #[cfg(feature = "dim3")]
        swept_bounds: local_aabb,
        linear_slop,
        max_fraction,
        best: None,
    };

    // The composite is stationary: every element sweep is degenerate at the composite pose
    // (composed with the child pose for compounds).
    let composite_sweep = Sweep::constant(composite_pose, Vector::ZERO);

    match typed {
        #[cfg(feature = "dim3")]
        TypedShape::TriMesh(mesh) => {
            let one_sided_mesh = mesh.flags().contains(TriMeshFlags::ORIENTED);
            context.one_sided = one_sided || one_sided_mesh;
            for tri_id in mesh.bvh().intersect_aabb(&local_aabb) {
                let triangle = mesh.triangle(tri_id);
                if context.one_sided_early_out(&triangle) {
                    continue;
                }
                let proxy = ToiProxy::from_array([triangle.a, triangle.b, triangle.c], 0.0);
                context.toi_against_element(&proxy, &composite_sweep);
            }
        }
        #[cfg(feature = "dim2")]
        TypedShape::TriMesh(mesh) => {
            for tri_id in mesh.bvh().intersect_aabb(&local_aabb) {
                let triangle = mesh.triangle(tri_id);
                let proxy = ToiProxy::from_array([triangle.a, triangle.b, triangle.c], 0.0);
                context.toi_against_element(&proxy, &composite_sweep);
            }
        }
        TypedShape::Polyline(polyline) => {
            #[cfg(feature = "dim2")]
            {
                let oriented = polyline.flags().contains(PolylineFlags::ORIENTED);
                context.one_sided = one_sided || oriented;
            }
            for seg_id in polyline.bvh().intersect_aabb(&local_aabb) {
                let segment = polyline.segment(seg_id);
                #[cfg(feature = "dim2")]
                if context.one_sided_early_out(&segment) {
                    continue;
                }
                let proxy = ToiProxy::from_array([segment.a, segment.b], 0.0);
                context.toi_against_element(&proxy, &composite_sweep);
            }
        }
        TypedShape::HeightField(heightfield) => {
            #[cfg(feature = "dim2")]
            heightfield.map_elements_in_local_aabb(&local_aabb, &mut |_, segment| {
                if !context.one_sided_early_out(segment) {
                    let proxy = ToiProxy::from_array([segment.a, segment.b], 0.0);
                    context.toi_against_element(&proxy, &composite_sweep);
                }
            });
            #[cfg(feature = "dim3")]
            heightfield.map_elements_in_local_aabb(&local_aabb, &mut |_, triangle| {
                if !context.one_sided_early_out(triangle) {
                    let proxy = ToiProxy::from_array([triangle.a, triangle.b, triangle.c], 0.0);
                    context.toi_against_element(&proxy, &composite_sweep);
                }
            });
        }
        TypedShape::Compound(compound) => {
            for child_id in compound.bvh().intersect_aabb(&local_aabb) {
                let (child_pose, child_shape) = &compound.shapes()[child_id as usize];
                let child_world_pose = *composite_pose * *child_pose;
                if let Some(child_proxy) = ToiProxy::from_shape(child_shape.as_ref()) {
                    let child_sweep = Sweep::constant(&child_world_pose, Vector::ZERO);
                    context.toi_against_element(&child_proxy, &child_sweep);
                } else if let Some(hit) = sweep_time_of_impact_composite(
                    child_shape.as_ref(),
                    &child_world_pose,
                    context.fast,
                    context.one_sided,
                    target_is_sensor,
                    context.max_fraction,
                    linear_slop,
                ) {
                    if 0.0 < hit.fraction && hit.fraction < context.max_fraction {
                        context.max_fraction = hit.fraction;
                        context.best = Some(hit);
                    }
                }
                // Children that are neither point-cloud shapes nor composites are skipped.
            }
        }
        _ => return None,
    }

    Some(context.best.unwrap_or(SweepToiOutput {
        status: SweepToiStatus::Separated,
        fraction: max_fraction,
        point: Vector::ZERO,
        normal: Vector::ZERO,
    }))
}

#[cfg(all(test, feature = "dim3"))]
mod rotating_bounds_tests {
    use super::*;
    use crate::{
        math::Rotation,
        shape::{Cuboid, HeightField},
        utils::Array2,
    };

    #[test]
    fn stationary_centroid_does_not_hide_a_rotating_tip() {
        let floor = HeightField::new(Array2::repeat(3, 3, 0.0), Vector::new(10.0, 1.0, 10.0));
        let bar = Cuboid::new(Vector::new(2.0, 0.01, 0.1));
        let proxy = ToiProxy::from_shape(&bar).unwrap();
        let start = Pose::translation(0.0, 1.0, 0.0);
        let angle = (40.0 as Real).to_radians();
        let end = Pose::from_parts(start.translation, Rotation::from_rotation_z(angle));
        assert!(1.0 - bar.half_extents.y > 0.0);
        assert!(1.0 - 2.0 * angle.sin() - 0.01 * angle.cos() < 0.0);
        let sweep = Sweep::from_poses(&start, &end, Vector::ZERO);
        let fast = SweepCompositeFastShape {
            proxy: &proxy,
            sweep: &sweep,
            local_centroid: Vector::ZERO,
            min_extent: bar.ccd_thickness(),
        };
        for one_sided in [false, true] {
            let hit = sweep_time_of_impact_composite(
                &floor,
                &Pose::IDENTITY,
                fast,
                one_sided,
                false,
                1.0,
                0.0001,
            )
            .unwrap();
            assert_eq!(hit.status, SweepToiStatus::Hit, "{hit:?}");
            assert!(hit.fraction > 0.0 && hit.fraction < 1.0);
            let clipped = sweep_time_of_impact_composite(
                &floor,
                &Pose::IDENTITY,
                fast,
                one_sided,
                false,
                0.3,
                0.0001,
            )
            .unwrap();
            assert_eq!(clipped.status, SweepToiStatus::Separated);
        }
    }

    #[test]
    fn swept_bounds_contain_off_center_rotational_arcs() {
        let points = [Vector::new(-2.0, -0.01, -0.1), Vector::new(2.0, 0.01, 0.1)];
        let proxy = ToiProxy::from_points(&points, 0.02);
        for axis in [
            Vector::X,
            Vector::Y,
            Vector::Z,
            Vector::new(1.0, 2.0, -3.0).normalize(),
        ] {
            for angle in [0.0, 0.001, 0.1, 1.0, 3.0] {
                for local_center in [Vector::ZERO, Vector::new(4.0, -3.0, 2.0)] {
                    let start = Pose::from_parts(
                        Vector::new(10.0, -4.0, 3.0),
                        Rotation::from_axis_angle(axis, 0.7),
                    );
                    let end = Pose::from_parts(
                        Vector::new(10.5, -3.8, 2.7),
                        Rotation::from_axis_angle(axis, 0.7 + angle),
                    );
                    let sweep = Sweep::from_poses(&start, &end, local_center);
                    for fraction in [0.1, 0.5, 1.0] {
                        let bounds = swept_proxy_aabb(&proxy, &sweep, fraction);
                        for sample in 0..=100 {
                            let actual = proxy.compute_aabb(
                                &sweep.transform_at(fraction * sample as Real / 100.0),
                            );
                            // Rounding allowance only for evaluating the independent sampled boxes.
                            assert!(bounds
                                .loosened(32.0 * Real::EPSILON * 16.0)
                                .contains(&actual));
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn translation_only_bounds_remain_tight() {
        let proxy = ToiProxy::from_array(
            [Vector::new(-2.0, -0.01, 0.0), Vector::new(2.0, 0.01, 0.0)],
            0.1,
        );
        let sweep = Sweep::from_poses(
            &Pose::IDENTITY,
            &Pose::translation(3.0, -4.0, 2.0),
            Vector::ZERO,
        );
        let bounds = swept_proxy_aabb(&proxy, &sweep, 1.0);
        let endpoints = proxy
            .compute_aabb(&sweep.transform_at(0.0))
            .merged(&proxy.compute_aabb(&sweep.transform_at(1.0)));
        // Adding the proxy radius before/after translation can round differently.
        // Require containment and tightness within only that arithmetic allowance.
        let rounding = 8.0 * Real::EPSILON;
        assert!(bounds.loosened(rounding).contains(&endpoints));
        assert!((bounds.mins - endpoints.mins).abs().max_element() <= rounding);
        assert!((bounds.maxs - endpoints.maxs).abs().max_element() <= rounding);
    }
}
