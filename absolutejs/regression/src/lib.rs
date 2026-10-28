//! Exercise the canonical library source through its public simplex API.
macro_rules! regression {
    ($name:ident,$backend:ident) => {
        #[cfg(test)]
        mod $name {
            use $backend::{
                math::{Real, Vector},
                query,
            };
            #[test]
            fn all_replaced_face_vertices_preserve_previous_simplex() {
                use query::gjk::{CsoPoint, VoronoiSimplex};
                for replaced in 0..3 {
                    let vertices = [
                        Vector::new(-2.0, 1.0, -2.0),
                        Vector::new(2.0, 1.0, -2.0),
                        Vector::new(0.0, 1.0, 2.0),
                    ];
                    let mut simplex = VoronoiSimplex::new();
                    simplex.reset(CsoPoint::single_point(vertices[0]));
                    simplex.project_origin_and_reduce();
                    for point in &vertices[1..] {
                        assert!(simplex.add_point(CsoPoint::single_point(*point)));
                        simplex.project_origin_and_reduce();
                    }
                    assert_eq!(simplex.dimension(), 2);
                    let previous = (0..3)
                        .map(|i| (*simplex.point(i), simplex.proj_coord(i)))
                        .collect::<Vec<_>>();
                    let mut replacement = vertices[replaced];
                    replacement.y = 0.5;
                    assert!(simplex.add_point(CsoPoint::single_point(replacement)));
                    simplex.project_origin_and_reduce();
                    assert_eq!(simplex.dimension(), 2);
                    for (i, (point, weight)) in previous.into_iter().enumerate() {
                        assert_eq!(
                            *simplex.prev_point(i),
                            point,
                            "face {replaced}: previous vertex {i}"
                        );
                        assert_eq!(simplex.prev_proj_coord(i), weight);
                    }
                }
            }
            #[test]
            fn tetrahedron_reduction_preserves_previous_support_points() {
                use query::gjk::{CsoPoint, VoronoiSimplex};
                let points = [
                    [-3220.0454, -0.0005722046, 3298.322],
                    [-3395.2175, -0.00030517578, -3109.0085],
                    [3179.9546, -0.0005722046, -3101.678],
                    [-3395.2175, -0.00030517578, 3290.9915],
                ]
                .map(|p| {
                    CsoPoint::single_point(Vector::new(p[0] as Real, p[1] as Real, p[2] as Real))
                });
                let mut simplex = VoronoiSimplex::new();
                simplex.reset(points[0]);
                simplex.project_origin_and_reduce();
                for point in &points[1..3] {
                    assert!(simplex.add_point(*point));
                    simplex.project_origin_and_reduce();
                }
                assert_eq!(simplex.dimension(), 2);
                let previous = (0..=simplex.dimension())
                    .map(|i| (*simplex.point(i), simplex.proj_coord(i)))
                    .collect::<Vec<_>>();
                assert!(simplex.add_point(points[3]));
                simplex.project_origin_and_reduce();
                assert_eq!(simplex.dimension(), 2);
                for (i, (point, weight)) in previous.into_iter().enumerate() {
                    assert_eq!(
                        *simplex.prev_point(i),
                        point,
                        "previous support point {i} was overwritten"
                    );
                    assert_eq!(simplex.prev_proj_coord(i), weight);
                }
            }
        }
    };
}
regression!(single, parry3d);
regression!(double, parry3d_f64);

#[cfg(test)]
mod inertia;
