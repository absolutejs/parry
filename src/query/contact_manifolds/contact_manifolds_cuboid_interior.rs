use crate::math::{Pose, Real, Vector};
use crate::query::{ContactManifold, TrackedContact};
use crate::shape::{
    ConvexPolyhedron, Cuboid, PackedFeatureId, PolygonalFeature, PolygonalFeatureMap,
};

/// Use a box's exact face plane only when the entire convex hull lies within
/// that face's interior column. Edge/corner, contained and deep contacts fall
/// back to the general convex query without changing its cached manifold.
pub(crate) fn contact_manifold_cuboid_interior<ManifoldData, ContactData: Default + Copy>(
    pos12: &Pose,
    cuboid: &Cuboid,
    polyhedron: &ConvexPolyhedron,
    prediction: Real,
    manifold: &mut ContactManifold<ManifoldData, ContactData>,
    flipped: bool,
) -> bool {
    let mut low = Vector::splat(Real::INFINITY);
    let mut high = Vector::splat(Real::NEG_INFINITY);
    for point in polyhedron.points() {
        let p = pos12 * *point;
        low = low.min(p);
        high = high.max(p);
    }
    if !low.is_finite() || !high.is_finite() || !prediction.is_finite() {
        return false;
    }
    let half = cuboid.half_extents;
    for axis in 0..3 {
        for sign in [-1.0, 1.0] {
            let (near, far) = if sign > 0.0 {
                (low[axis], high[axis])
            } else {
                (-high[axis], -low[axis])
            };
            // The hull bounds' center must be outside this face, and the hull
            // may not cross the opposite face. This excludes embedded bodies.
            if near * 0.5 + far * 0.5 <= half[axis] || near <= -half[axis] {
                continue;
            }
            let clearance = prediction.max(0.0) + (half[axis] - near).max(0.0);
            if (0..3)
                .filter(|j| *j != axis)
                .any(|j| low[j] + half[j] <= clearance || half[j] - high[j] <= clearance)
            {
                continue;
            }
            let mut normal = Vector::ZERO;
            normal[axis] = sign;
            let local_normal = pos12.rotation.inverse() * normal;
            let mut feature = PolygonalFeature::default();
            polyhedron.local_support_feature(-local_normal, &mut feature);
            let face_id = cuboid.support_face(normal).fid;
            let old = core::mem::take(&mut manifold.points);
            for i in 0..feature.num_vertices {
                let local = feature.vertices[i];
                let world = pos12 * local;
                let distance = world.dot(normal) - half[axis];
                if distance <= prediction {
                    manifold.points.push(TrackedContact::flipped(
                        world - normal * distance,
                        local,
                        face_id,
                        feature.vids[i],
                        distance,
                        flipped,
                    ));
                }
            }
            // Normal alignment alone can choose a face away from the extreme
            // point of a long anisotropic hull. Preserve the true supporting
            // vertex even when it is absent from the selected/truncated face.
            let support_id =
                crate::utils::point_cloud_support_point_id(-local_normal, polyhedron.points());
            let support = polyhedron.points()[support_id];
            if !manifold
                .points
                .iter()
                .any(|p| (if flipped { p.local_p1 } else { p.local_p2 }) == support)
            {
                let world = pos12 * support;
                let distance = world.dot(normal) - half[axis];
                if distance <= prediction {
                    manifold.points.push(TrackedContact::flipped(
                        world - normal * distance,
                        support,
                        face_id,
                        PackedFeatureId::vertex(support_id as u32),
                        distance,
                        flipped,
                    ));
                }
            }
            if flipped {
                manifold.local_n1 = -local_normal;
                manifold.local_n2 = normal;
            } else {
                manifold.local_n1 = normal;
                manifold.local_n2 = -local_normal;
            }
            manifold.match_contacts(&old);
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;
    fn hull() -> ConvexPolyhedron {
        let points: Vec<_> = [-1.0, 1.0]
            .into_iter()
            .flat_map(|x| {
                [-1.0, 1.0]
                    .into_iter()
                    .flat_map(move |y| [-1.0, 1.0].into_iter().map(move |z| Vector::new(x, y, z)))
            })
            .collect();
        ConvexPolyhedron::from_convex_hull(&points).unwrap()
    }
    #[test]
    fn interior_face_has_exact_plane_normal_and_box_local_anchors() {
        let poly = hull();
        for axis in 0..3 {
            for sign in [-1.0, 1.0] {
                let b = Cuboid::new(Vector::splat(10.0));
                let mut t = Vector::ZERO;
                t[axis] = sign * 10.99;
                let pose = Pose::from_translation(t);
                for flipped in [false, true] {
                    let mut m = ContactManifold::<(), ()>::new();
                    assert!(contact_manifold_cuboid_interior(
                        &pose, &b, &poly, 0.1, &mut m, flipped
                    ));
                    assert_eq!(m.points.len(), 4);
                    let mut normal = Vector::ZERO;
                    normal[axis] = sign;
                    assert_eq!(if flipped { m.local_n2 } else { m.local_n1 }, normal);
                    for p in &m.points {
                        let box_point = if flipped { p.local_p2 } else { p.local_p1 };
                        assert_eq!(box_point[axis], sign * 10.0);
                        assert!((p.dist + 0.01).abs() < 1e-5);
                    }
                }
            }
        }
    }
    #[test]
    fn edge_corner_embedded_and_opposite_face_cases_keep_fallback_untouched() {
        let poly = hull();
        let b = Cuboid::new(Vector::splat(10.0));
        for t in [
            Vector::new(9.1, 10.99, 0.0),
            Vector::new(9.1, 10.99, 9.1),
            Vector::ZERO,
            Vector::new(0.0, 9.0, 0.0),
        ] {
            let mut m = ContactManifold::<(), ()>::new();
            m.local_n1 = Vector::Z;
            assert!(!contact_manifold_cuboid_interior(
                &Pose::from_translation(t),
                &b,
                &poly,
                0.1,
                &mut m,
                false
            ));
            assert!(m.points.is_empty());
            assert_eq!(m.local_n1, Vector::Z);
        }
    }
    #[test]
    fn separated_interior_column_has_no_speculative_contact_beyond_prediction() {
        let mut m = ContactManifold::<(), ()>::new();
        assert!(contact_manifold_cuboid_interior(
            &Pose::from_translation(Vector::new(0.0, 15.0, 0.0)),
            &Cuboid::new(Vector::splat(10.0)),
            &hull(),
            0.1,
            &mut m,
            false
        ));
        assert!(m.points.is_empty());
    }
    #[test]
    fn rotated_hull_contacts_keep_world_plane_and_feature_ids() {
        use crate::math::Rot3;
        let poly = hull();
        let rotation = Rot3::from_axis_angle(Vector::new(1.0, 2.0, 3.0).normalize(), 0.7);
        let pose = Pose::from_parts(Vector::new(0.0, 11.0, 0.0), rotation);
        let mut m = ContactManifold::<(), ()>::new();
        assert!(contact_manifold_cuboid_interior(
            &pose,
            &Cuboid::new(Vector::splat(10.0)),
            &poly,
            0.5,
            &mut m,
            false
        ));
        assert!(!m.points.is_empty());
        assert_eq!(m.local_n1, Vector::Y);
        assert!((pose.rotation * m.local_n2 + Vector::Y).length() < 1e-6);
        for p in &m.points {
            assert_eq!(p.local_p1.y, 10.0);
            assert!(((pose * p.local_p2 - p.local_p1).dot(Vector::Y) - p.dist).abs() < 1e-5);
        }
    }
    #[test]
    fn dispatcher_preserves_contact_identity_for_both_shape_orders() {
        use crate::query::{DefaultQueryDispatcher, PersistentQueryDispatcher};
        let poly = hull();
        let cube = Cuboid::new(Vector::splat(10.0));
        let pose = Pose::translation(0.0, 10.99, 0.0);
        for flipped in [false, true] {
            let mut m = ContactManifold::<(), u32>::new();
            for iteration in 0..2 {
                if flipped {
                    DefaultQueryDispatcher
                        .contact_manifold_convex_convex(
                            &pose.inverse(),
                            &poly,
                            &cube,
                            None,
                            None,
                            0.1,
                            &mut m,
                        )
                        .unwrap();
                } else {
                    DefaultQueryDispatcher
                        .contact_manifold_convex_convex(
                            &pose, &cube, &poly, None, None, 0.1, &mut m,
                        )
                        .unwrap();
                }
                assert_eq!(m.points.len(), 4);
                for p in &mut m.points {
                    if iteration == 1 {
                        assert_eq!(p.data, 42);
                    }
                    p.data = 42;
                }
            }
        }
    }
    #[test]
    fn anisotropic_rotated_hulls_retain_the_deepest_vertex() {
        use crate::math::Rot3;
        use crate::shape::SupportMap;
        let points: Vec<_> = (0..32)
            .map(|i| {
                let y = 1.0 - 2.0 * (i as Real + 0.5) / 32.0;
                let radius = (1.0 - y * y).sqrt();
                let angle = i as Real * 2.399963229728653;
                Vector::new(4.0 * radius * angle.cos(), 0.2 * y, radius * angle.sin())
            })
            .collect();
        let poly = ConvexPolyhedron::from_convex_hull(&points).unwrap();
        let cube = Cuboid::new(Vector::splat(10.0));
        for i in 0..96 {
            let rotation =
                Rot3::from_axis_angle(Vector::new(1.0, 2.0, 3.0).normalize(), i as Real * 0.065);
            let support = poly.local_support_point(rotation.inverse() * -Vector::Y);
            let translation = Vector::new(0.0, 10.0 - (rotation * support).y - 0.01, 0.0);
            let pose = Pose::from_parts(translation, rotation);
            let mut m = ContactManifold::<(), ()>::new();
            assert!(contact_manifold_cuboid_interior(
                &pose, &cube, &poly, 0.02, &mut m, false
            ));
            assert!(
                m.points.iter().any(|p| p.local_p2 == support),
                "rotation {i}: deepest support vertex missing from {} contacts",
                m.points.len()
            );
        }
    }
}
