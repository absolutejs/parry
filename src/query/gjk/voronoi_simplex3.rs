use crate::math::{Real, Vector};
use crate::query::gjk::{self, CsoPoint};
use crate::query::{PointQuery, PointQueryWithLocation};
use crate::shape::{
    Segment, SegmentPointLocation, Tetrahedron, TetrahedronPointLocation, Triangle,
    TrianglePointLocation,
};

/// A simplex of dimension up to 3 that uses Voronoï regions for computing point projections.
#[derive(Clone, Debug)]
pub struct VoronoiSimplex {
    prev_vertices: [usize; 4],
    prev_proj: [Real; 3],
    prev_dim: usize,

    vertices: [CsoPoint; 4],
    proj: [Real; 3],
    dim: usize,
}

impl Default for VoronoiSimplex {
    fn default() -> Self {
        Self::new()
    }
}

impl VoronoiSimplex {
    /// Creates a new empty simplex.
    pub fn new() -> VoronoiSimplex {
        VoronoiSimplex {
            prev_vertices: [0, 1, 2, 3],
            prev_proj: [0.0; 3],
            prev_dim: 0,
            vertices: [CsoPoint::ZERO; 4],
            proj: [0.0; 3],
            dim: 0,
        }
    }

    /// Swap two vertices of this simplex.
    pub fn swap(&mut self, i1: usize, i2: usize) {
        self.vertices.swap(i1, i2);
        self.prev_vertices.swap(i1, i2);
    }

    /// Resets this simplex to a single point.
    pub fn reset(&mut self, pt: CsoPoint) {
        self.dim = 0;
        self.prev_dim = 0;
        self.vertices[0] = pt;
    }

    /// Add a point to this simplex.
    pub fn add_point(&mut self, pt: CsoPoint) -> bool {
        self.prev_dim = self.dim;
        self.prev_proj = self.proj;
        self.prev_vertices = [0, 1, 2, 3];

        match self.dim {
            0 => {
                if (self.vertices[0] - pt).length_squared() < gjk::eps_tol() {
                    return false;
                }
            }
            1 => {
                let ab = self.vertices[1] - self.vertices[0];
                let ac = pt - self.vertices[0];

                if ab.cross(ac).length_squared() < gjk::eps_tol() {
                    return false;
                }
            }
            2 => {
                let ab = self.vertices[1] - self.vertices[0];
                let ac = self.vertices[2] - self.vertices[0];
                let ap = pt - self.vertices[0];
                let n = ab.cross(ac).normalize();

                if n.dot(ap).abs() < gjk::eps_tol() {
                    return false;
                }
            }
            _ => unreachable!(),
        }

        self.dim += 1;
        self.vertices[self.dim] = pt;
        true
    }

    /// Retrieves the barycentric coordinate associated to the `i`-th by the last call to `project_origin_and_reduce`.
    pub fn proj_coord(&self, i: usize) -> Real {
        assert!(i <= self.dim, "Index out of bounds.");
        self.proj[i]
    }

    /// The i-th point of this simplex.
    pub fn point(&self, i: usize) -> &CsoPoint {
        assert!(i <= self.dim, "Index out of bounds.");
        &self.vertices[i]
    }

    /// Retrieves the barycentric coordinate associated to the `i`-th before the last call to `project_origin_and_reduce`.
    pub fn prev_proj_coord(&self, i: usize) -> Real {
        assert!(i <= self.prev_dim, "Index out of bounds.");
        self.prev_proj[i]
    }

    /// The i-th point of the simplex before the last call to `project_origin_and_reduce`.
    pub fn prev_point(&self, i: usize) -> &CsoPoint {
        assert!(i <= self.prev_dim, "Index out of bounds.");
        &self.vertices[self.prev_vertices[i]]
    }

    /// Projects the origin on the boundary of this simplex and reduces `self` the smallest subsimplex containing the origin.
    ///
    /// Returns the result of the projection or `Vector::ZERO` if the origin lies inside of the simplex.
    /// The state of the simplex before projection is saved, and can be retrieved using the methods prefixed
    /// by `prev_`.
    pub fn project_origin_and_reduce(&mut self) -> Vector {
        if self.dim == 0 {
            self.proj[0] = 1.0;
            self.vertices[0].point
        } else if self.dim == 1 {
            let (proj, location) = Segment::new(self.vertices[0].point, self.vertices[1].point)
                .project_local_point_and_get_location(Vector::ZERO, true);

            match location {
                SegmentPointLocation::OnVertex(0) => {
                    self.proj[0] = 1.0;
                    self.dim = 0;
                }
                SegmentPointLocation::OnVertex(1) => {
                    self.swap(0, 1);
                    self.proj[0] = 1.0;
                    self.dim = 0;
                }
                SegmentPointLocation::OnEdge(coords) => {
                    self.proj[0] = coords[0];
                    self.proj[1] = coords[1];
                }
                _ => unreachable!(),
            }

            proj.point
        } else if self.dim == 2 {
            let (proj, location) = Triangle::new(
                self.vertices[0].point,
                self.vertices[1].point,
                self.vertices[2].point,
            )
            .project_local_point_and_get_location(Vector::ZERO, true);

            match location {
                TrianglePointLocation::OnVertex(i) => {
                    self.swap(0, i as usize);
                    self.proj[0] = 1.0;
                    self.dim = 0;
                }
                TrianglePointLocation::OnEdge(0, coords) => {
                    self.proj[0] = coords[0];
                    self.proj[1] = coords[1];
                    self.dim = 1;
                }
                TrianglePointLocation::OnEdge(1, coords) => {
                    self.swap(0, 2);
                    self.proj[0] = coords[1];
                    self.proj[1] = coords[0];
                    self.dim = 1;
                }
                TrianglePointLocation::OnEdge(2, coords) => {
                    self.swap(1, 2);
                    self.proj[0] = coords[0];
                    self.proj[1] = coords[1];
                    self.dim = 1;
                }
                TrianglePointLocation::OnFace(_, coords) => {
                    self.proj = coords;
                }
                _ => {}
            }

            proj.point
        } else {
            assert!(self.dim == 3);
            let (proj, location) = Tetrahedron::new(
                self.vertices[0].point,
                self.vertices[1].point,
                self.vertices[2].point,
                self.vertices[3].point,
            )
            .project_local_point_and_get_location(Vector::ZERO, true);

            match location {
                TetrahedronPointLocation::OnVertex(i) => {
                    self.swap(0, i as usize);
                    self.proj[0] = 1.0;
                    self.dim = 0;
                }
                TetrahedronPointLocation::OnEdge(i, coords) => {
                    match i {
                        0 => {
                            // ab
                        }
                        1 => {
                            // ac
                            self.swap(1, 2)
                        }
                        2 => {
                            // ad
                            self.swap(1, 3)
                        }
                        3 => {
                            // bc
                            self.swap(0, 2)
                        }
                        4 => {
                            // bd
                            self.swap(0, 3)
                        }
                        5 => {
                            // cd
                            self.swap(0, 2);
                            self.swap(1, 3);
                        }
                        _ => unreachable!(),
                    }

                    match i {
                        0 | 1 | 2 | 5 => {
                            self.proj[0] = coords[0];
                            self.proj[1] = coords[1];
                        }
                        3 | 4 => {
                            self.proj[0] = coords[1];
                            self.proj[1] = coords[0];
                        }
                        _ => unreachable!(),
                    }
                    self.dim = 1;
                }
                TetrahedronPointLocation::OnFace(i, coords) => {
                    match i {
                        0 => {
                            // abc
                            self.proj = coords;
                        }
                        1 => {
                            // Keep discarded vertices available to prev_point: GJK may
                            // return the previous simplex if its upper bound regresses.
                            // abd
                            self.swap(2, 3);
                            self.proj = coords;
                        }
                        2 => {
                            // acd
                            self.swap(1, 3);
                            self.proj[0] = coords[0];
                            self.proj[1] = coords[2];
                            self.proj[2] = coords[1];
                        }
                        3 => {
                            // bcd
                            self.swap(0, 3);
                            self.proj[0] = coords[2];
                            self.proj[1] = coords[0];
                            self.proj[2] = coords[1];
                        }
                        _ => unreachable!(),
                    }
                    self.dim = 2;
                }
                _ => {}
            }

            proj.point
        }
    }

    /// Compute the projection of the origin on the boundary of this simplex.
    pub fn project_origin(&mut self) -> Vector {
        if self.dim == 0 {
            self.vertices[0].point
        } else if self.dim == 1 {
            let seg = Segment::new(self.vertices[0].point, self.vertices[1].point);
            seg.project_local_point(Vector::ZERO, true).point
        } else if self.dim == 2 {
            let tri = Triangle::new(
                self.vertices[0].point,
                self.vertices[1].point,
                self.vertices[2].point,
            );
            tri.project_local_point(Vector::ZERO, true).point
        } else {
            let tetr = Tetrahedron::new(
                self.vertices[0].point,
                self.vertices[1].point,
                self.vertices[2].point,
                self.vertices[3].point,
            );
            tetr.project_local_point(Vector::ZERO, true).point
        }
    }

    /// Tests if the given point is already a vertex of this simplex.
    pub fn contains_point(&self, pt: Vector) -> bool {
        for i in 0..self.dim + 1 {
            if self.vertices[i].point == pt {
                return true;
            }
        }

        false
    }

    /// The dimension of the smallest subspace that can contain this simplex.
    pub fn dimension(&self) -> usize {
        self.dim
    }

    /// The dimension of the simplex before the last call to `project_origin_and_reduce`.
    pub fn prev_dimension(&self) -> usize {
        self.prev_dim
    }

    /// The maximum squared length of the vertices of this simplex.
    pub fn max_sq_len(&self) -> Real {
        let mut max_sq_len = 0.0;

        for i in 0..self.dim + 1 {
            let norm = self.vertices[i].point.length_squared();

            if norm > max_sq_len {
                max_sq_len = norm
            }
        }

        max_sq_len
    }

    /// Apply a function to all the vertices of this simplex.
    pub fn modify_pnts(&mut self, f: &dyn Fn(&mut CsoPoint)) {
        for i in 0..self.dim + 1 {
            f(&mut self.vertices[i])
        }
    }
}

#[cfg(test)]
mod witness_regression_tests {
    use crate::math::{Real, Vector};
    #[test]
    fn all_replaced_face_vertices_preserve_previous_simplex() {
        use crate::query::gjk::{CsoPoint, VoronoiSimplex};
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
        use crate::query::gjk::{CsoPoint, VoronoiSimplex};
        let points = [
            [-3220.0454, -0.0005722046, 3298.322],
            [-3395.2175, -0.00030517578, -3109.0085],
            [3179.9546, -0.0005722046, -3101.678],
            [-3395.2175, -0.00030517578, 3290.9915],
        ]
        .map(|p| CsoPoint::single_point(Vector::new(p[0] as Real, p[1] as Real, p[2] as Real)));
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
