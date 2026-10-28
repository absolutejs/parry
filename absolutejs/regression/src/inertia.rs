macro_rules! inertia_regression {
 ($name:ident, $backend:ident, $tolerance:expr) => {
  #[cfg(test)] mod $name {
   use $backend::{mass_properties::MassProperties, math::{Matrix, Real, Rotation, Vector}};
   fn relative_error(actual: Matrix, expected: Matrix) -> f64 {
    let a=actual.to_cols_array(); let b=expected.to_cols_array();
    let scale=b.iter().map(|v|(*v as f64).abs()).fold(0.0_f64,f64::max).max(f64::MIN_POSITIVE);
    a.iter().zip(b).map(|(a,b)|((*a as f64-b as f64)/scale).powi(2)).sum::<f64>().sqrt()
   }
   fn orientation(i:usize)->Rotation {
    Rotation::from_xyzw((0.17+i as f64*0.03) as Real,(-0.41+i as f64*0.01) as Real,0.23,0.83).normalize()
   }
   #[test] fn tensor_reconstruction_across_scale_rotation_and_repeated_moments() {
    let mut maximum=0.0_f64; let mut count=0;
    for scale in [1e-24,1e-12,1.0,1e12,1e24] {
     for diagonal in [[1.,2.,3.],[1.,1.,2.],[1.,2.,2.],[1.,1.,1.],[1.,1.+1e-7,2.],[1e-8,1.,1.]] {
      for i in 0..32 {
       let frame=Matrix::from_quat(orientation(i));
       let expected=frame*Matrix::from_diagonal(Vector::new((diagonal[0]*scale) as Real,(diagonal[1]*scale) as Real,(diagonal[2]*scale) as Real))*frame.transpose();
       let p=MassProperties::with_inertia_matrix(Vector::ZERO,1.0,expected);
       let error=relative_error(p.reconstruct_inertia_matrix(),expected);
       assert!(error.is_finite() && error < $tolerance,"scale={scale} diagonal={diagonal:?} orientation={i} error={error:e}");
       assert!(p.principal_inertia_local_frame.is_finite());
       maximum=maximum.max(error); count+=1;
      }
     }
    }
    println!("{} tensor cases={count} maximum_relative_error={maximum:e}",stringify!($name));
   }
   #[test] fn aggregation_matches_parallel_axis_theorem_for_each_insertion_order() {
    let masses=[2.0,3.0,4.0];
    let positions=[Vector::ZERO,Vector::new(0.65,0.15,0.1),Vector::new(0.2,0.5,-0.1)];
    let mut maximum=0.0_f64;
    for units in [0.001,1.0,100.0,10000.0] {
     for i in 0..16 {
      let q=orientation(i);
      let props:Vec<_>=(0..3).map(|k| MassProperties::new(q*positions[k]*units,masses[k],Vector::splat(0.4*masses[k]*(0.1*units).powi(2)))).collect();
      let com=props.iter().zip(masses).map(|(p,m)|p.local_com*m).sum::<Vector>()/9.0;
      let expected=props.iter().zip(masses).map(|(p,m)|{
       let r=p.local_com-com;
       p.reconstruct_inertia_matrix()+(Matrix::IDENTITY*r.length_squared()-Matrix::from_cols(r*r.x,r*r.y,r*r.z))*m
      }).fold(Matrix::ZERO,|a,b|a+b);
      for order in [[0,1,2],[0,2,1],[1,0,2],[1,2,0],[2,0,1],[2,1,0]] {
       let aggregate=order.into_iter().fold(MassProperties::default(),|a,k|a+props[k]);
       let error=relative_error(aggregate.reconstruct_inertia_matrix(),expected);
       assert!(error.is_finite() && error<$tolerance*2.0,"units={units} order={order:?} error={error:e}");
       maximum=maximum.max(error);
      }
     }
    }
    println!("{} aggregate cases=384 maximum_relative_error={maximum:e}",stringify!($name));
   }
   #[test] fn zero_diagonal_and_subtraction_preserve_physical_tensor() {
    let zero=MassProperties::with_inertia_matrix(Vector::ZERO,0.0,Matrix::ZERO);
    assert_eq!(zero.reconstruct_inertia_matrix(),Matrix::ZERO);
    let a=MassProperties::new(Vector::ZERO,2.0,Vector::new(1.0,2.0,3.0));
    let b=MassProperties::new(Vector::new(0.2,0.3,-0.1),1.0,Vector::new(0.4,0.5,0.6));
    assert!(relative_error(((a+b)-b).reconstruct_inertia_matrix(),a.reconstruct_inertia_matrix())<$tolerance*2.0);
    assert!(relative_error(((a+b)-a).reconstruct_inertia_matrix(),b.reconstruct_inertia_matrix())<$tolerance*4.0);
   }
  }
 };
}
inertia_regression!(inertia_f32, parry3d, 2e-6);
inertia_regression!(inertia_f64, parry3d_f64, 2e-14);
