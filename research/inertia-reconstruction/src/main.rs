use parry3d::mass_properties::MassProperties;
use parry3d::math::{Matrix, Rotation, Vector};
fn norm(m: Matrix) -> f32 {
    m.to_cols_array()
        .into_iter()
        .map(|x| x * x)
        .sum::<f32>()
        .sqrt()
}
fn main() {
    let orientation = Rotation::from_xyzw(0.2, 0.3, 0.1, 0.9).normalize();
    let offsets = [
        Vector::ZERO,
        Vector::new(0.65, 0.15, 0.1),
        Vector::new(0.2, 0.5, -0.1),
    ];
    let masses = [2.0, 3.0, 4.0];
    let mut maximum = 0.0f32;
    for units in [0.001f32, 1.0, 100.0, 10000.0] {
        for count in [2, 3] {
            let props: Vec<_> = (0..count)
                .map(|i| {
                    MassProperties::new(
                        orientation * offsets[i] * units,
                        masses[i],
                        Vector::splat(0.4 * masses[i] * (0.1 * units).powi(2)),
                    )
                })
                .collect();
            let mass: f32 = masses[..count].iter().sum();
            let com = props
                .iter()
                .zip(masses)
                .map(|(p, m)| p.local_com * m)
                .sum::<Vector>()
                / mass;
            let expected = props
                .iter()
                .zip(masses)
                .map(|(p, m)| {
                    let r = p.local_com - com;
                    p.reconstruct_inertia_matrix()
                        + (Matrix::IDENTITY * r.length_squared()
                            - Matrix::from_cols(r * r.x, r * r.y, r * r.z))
                            * m
                })
                .fold(Matrix::ZERO, |a, b| a + b);
            let aggregate = props
                .iter()
                .copied()
                .fold(MassProperties::default(), |a, b| a + b);
            let direct = MassProperties::with_inertia_matrix(com, mass, expected);
            let aggregate_error =
                norm(aggregate.reconstruct_inertia_matrix() - expected) / norm(expected);
            let direct_error =
                norm(direct.reconstruct_inertia_matrix() - expected) / norm(expected);
            maximum = maximum.max(aggregate_error).max(direct_error);
            println!("units={units} count={count} aggregate_relative_error={aggregate_error:e} direct_relative_error={direct_error:e}");
        }
    }
    assert!(
        maximum < 2e-6,
        "Inertia reconstruction error {maximum:e} exceeds f32 tolerance"
    );
}
