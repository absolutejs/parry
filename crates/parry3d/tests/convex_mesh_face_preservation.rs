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

#[test]
fn captured_cut_support_feature_contains_support_vertex() {
    use parry3d::shape::{PolygonalFeature, PolygonalFeatureMap, SupportMap};
    let points = vec![
        Vector::new(
            -2.591353416442871 as Real,
            -19.12188720703125 as Real,
            -2.1695542335510254 as Real,
        ),
        Vector::new(
            -3.1980228424072266 as Real,
            -18.156734466552734 as Real,
            -0.7643530368804932 as Real,
        ),
        Vector::new(
            -1.5901561975479126 as Real,
            -19.94156265258789 as Real,
            -3.2408699989318848 as Real,
        ),
        Vector::new(
            -0.346596896648407 as Real,
            -20.49121856689453 as Real,
            -3.8145861625671387 as Real,
        ),
        Vector::new(
            -3.3186516761779785 as Real,
            -17.19292640686035 as Real,
            0.7603739500045776 as Real,
        ),
        Vector::new(
            -2.9358174800872803 as Real,
            -16.377290725708008 as Real,
            2.172499418258667 as Real,
        ),
        Vector::new(
            -2.1082842350006104 as Real,
            -15.834250450134277 as Real,
            3.2574660778045654 as Real,
        ),
        Vector::new(
            -0.9617814421653748 as Real,
            -15.646740913391113 as Real,
            3.8506999015808105 as Real,
        ),
        Vector::new(
            -1.9090219736099243 as Real,
            22.068317413330078 as Real,
            -1.6044152975082397 as Real,
        ),
        Vector::new(
            -2.374336004257202 as Real,
            22.068317413330078 as Real,
            -0.5600235462188721 as Real,
        ),
        Vector::new(
            -2.2208895683288574 as Real,
            22.068317413330078 as Real,
            -0.9072313904762268 as Real,
        ),
        Vector::new(
            -2.4109621047973633 as Real,
            22.068317413330078 as Real,
            -0.1884697824716568 as Real,
        ),
        Vector::new(
            -1.161975622177124 as Real,
            22.068317413330078 as Real,
            -2.392054319381714 as Real,
        ),
        Vector::new(
            -0.8521149754524231 as Real,
            22.068317413330078 as Real,
            -2.5376622676849365 as Real,
        ),
        Vector::new(
            -0.24740047752857208 as Real,
            22.068317413330078 as Real,
            -2.810058116912842 as Real,
        ),
        Vector::new(
            -1.6590884923934937 as Real,
            22.068317413330078 as Real,
            -1.8726832866668701 as Real,
        ),
        Vector::new(
            -2.482487678527832 as Real,
            22.068317413330078 as Real,
            0.5862976312637329 as Real,
        ),
        Vector::new(
            -2.4002249240875244 as Real,
            22.068317413330078 as Real,
            0.9256550669670105 as Real,
        ),
        Vector::new(
            -2.210909605026245 as Real,
            22.068317413330078 as Real,
            1.6588058471679688 as Real,
        ),
        Vector::new(
            -2.025115966796875 as Real,
            22.068317413330078 as Real,
            1.916015863418579 as Real,
        ),
        Vector::new(
            -1.5969260931015015 as Real,
            22.068317413330078 as Real,
            2.488178014755249 as Real,
        ),
        Vector::new(
            -1.3371856212615967 as Real,
            22.068317413330078 as Real,
            2.6263411045074463 as Real,
        ),
        Vector::new(
            -0.7345467805862427 as Real,
            22.068317413330078 as Real,
            2.9408514499664307 as Real,
        ),
        Vector::new(
            -0.05433700606226921 as Real,
            22.068317413330078 as Real,
            2.9444739818573 as Real,
        ),
        Vector::new(
            0.9495158791542053 as Real,
            -20.687410354614258 as Real,
            -3.8029253482818604 as Real,
        ),
        Vector::new(
            2.0999178886413574 as Real,
            -20.500391006469727 as Real,
            -3.2076776027679443 as Real,
        ),
        Vector::new(
            2.9286201000213623 as Real,
            -19.958505630493164 as Real,
            -2.1198933124542236 as Real,
        ),
        Vector::new(
            3.3092074394226074 as Real,
            -19.144006729125977 as Real,
            -0.7057977914810181 as Real,
        ),
        Vector::new(
            0.329985648393631 as Real,
            -15.84339427947998 as Real,
            3.8623294830322266 as Real,
        ),
        Vector::new(
            1.571298599243164 as Real,
            -16.394187927246094 as Real,
            3.2905831336975098 as Real,
        ),
        Vector::new(
            3.1842329502105713 as Real,
            -18.180662155151367 as Real,
            0.8188983201980591 as Real,
        ),
        Vector::new(
            2.573664665222168 as Real,
            -17.21501922607422 as Real,
            2.222085475921631 as Real,
        ),
        Vector::new(
            0.07523337751626968 as Real,
            22.068317413330078 as Real,
            -2.8084607124328613 as Real,
        ),
        Vector::new(
            0.6992999911308289 as Real,
            22.068317413330078 as Real,
            -2.8006246089935303 as Real,
        ),
        Vector::new(
            1.254164695739746 as Real,
            22.068317413330078 as Real,
            -2.515908718109131 as Real,
        ),
        Vector::new(
            1.5398991107940674 as Real,
            22.068317413330078 as Real,
            -2.3665261268615723 as Real,
        ),
        Vector::new(
            1.9458190202713013 as Real,
            22.068317413330078 as Real,
            -1.8434454202651978 as Real,
        ),
        Vector::new(
            2.151043653488159 as Real,
            22.068317413330078 as Real,
            -1.5699701309204102 as Real,
        ),
        Vector::new(
            2.3457956314086914 as Real,
            22.068317413330078 as Real,
            -0.8791635632514954 as Real,
        ),
        Vector::new(
            2.440382242202759 as Real,
            22.068317413330078 as Real,
            -0.5252984762191772 as Real,
        ),
        Vector::new(
            0.24011290073394775 as Real,
            22.068317413330078 as Real,
            2.943732261657715 as Real,
        ),
        Vector::new(
            0.8872919082641602 as Real,
            22.068317413330078 as Real,
            2.6383633613586426 as Real,
        ),
        Vector::new(
            1.1724445819854736 as Real,
            22.068317413330078 as Real,
            2.4978902339935303 as Real,
        ),
        Vector::new(
            2.387981414794922 as Real,
            22.068317413330078 as Real,
            0.23592962324619293 as Real,
        ),
        Vector::new(
            2.2177207469940186 as Real,
            22.068317413330078 as Real,
            0.9599357843399048 as Real,
        ),
        Vector::new(
            2.3601839542388916 as Real,
            22.068317413330078 as Real,
            0.6144746541976929 as Real,
        ),
        Vector::new(
            1.6842964887619019 as Real,
            22.068317413330078 as Real,
            1.938882827758789 as Real,
        ),
        Vector::new(
            1.9166468381881714 as Real,
            22.068317413330078 as Real,
            1.6774344444274902 as Real,
        ),
    ];
    let shape = ConvexPolyhedron::from_convex_hull(&points).unwrap();
    let dir = Vector::new(
        0.09866894006578407 as Real,
        -0.9059932769705175 as Real,
        -0.41163166795287304 as Real,
    );
    let mut feature = PolygonalFeature::default();
    shape.local_support_feature(dir, &mut feature);
    let support = shape.local_support_point(dir).dot(dir);
    let feature_support = feature.vertices[..feature.num_vertices]
        .iter()
        .map(|p| p.dot(dir))
        .fold(Real::NEG_INFINITY, Real::max);
    println!(
        "faces={} feature_vertices={} support={} feature_support={} deficit={}",
        shape.faces().len(),
        feature.num_vertices,
        support,
        feature_support,
        support - feature_support
    );
    assert!((support - feature_support).abs() <= 1e-5);
}
