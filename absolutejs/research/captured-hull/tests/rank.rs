use parry3d_f64::{math::Vector,transformation::try_convex_hull};
fn certify(points:&[Vector]) {
 let (vertices,faces)=try_convex_hull(points).expect("Hull rejected");
 let coord=|p:Vector|robust::Coord3D{x:p.x,y:p.y,z:p.z};
 let mut volume=0.0;
 for face in faces{
  let a=vertices[face[0] as usize];let b=vertices[face[1] as usize];let c=vertices[face[2] as usize];
  for point in points{assert!(robust::orient3d(coord(a),coord(b),coord(c),coord(*point))>=0.0,"Input outside face");}
  volume+=a.dot(b.cross(c))/6.0;
 }
 assert!(volume>0.0,"Not a positive solid");
 assert!(vertices.iter().all(|p|points.contains(p)));
}
#[test]
fn thin_tetrahedra_are_not_planar(){
 for height in [1e-6,1e-9] {for axis in 0..3 {
  let points=[Vector::ZERO,Vector::X,Vector::Y,Vector::new(0.25,0.25,height)].map(|p|Vector::new(p[axis],p[(axis+1)%3],p[(axis+2)%3]));certify(&points);
 }}
}
#[test]
fn truly_planar_points_keep_zero_volume(){
 let p=[Vector::ZERO,Vector::X,Vector::Y,Vector::new(1.0,1.0,0.0)];let(v,i)=try_convex_hull(&p).unwrap();
 assert!(v.iter().all(|p|p.z==0.0));assert_eq!(i.len(),4);
 assert_eq!(i.iter().map(|t|v[t[0] as usize].dot(v[t[1] as usize].cross(v[t[2] as usize]))).sum::<f64>(),0.0);
}
#[test]
fn captured_thin_exports_keep_their_volume(){
 for text in [include_str!("../rank-fixtures/input-25.txt"),include_str!("../rank-fixtures/input-26.txt"),include_str!("../rank-fixtures/input-67.txt"),include_str!("../rank-fixtures/input-68.txt"),include_str!("../rank-fixtures/input-132.txt")]{
  let bits:Vec<u32>=text.split_whitespace().map(|s|s.parse().unwrap()).collect();let p:Vec<_>=bits.chunks_exact(3).map(|p|Vector::new(f32::from_bits(p[0]) as f64,f32::from_bits(p[1]) as f64,f32::from_bits(p[2]) as f64)).collect();certify(&p);
 }
}
#[path="../src/input.rs"] mod input;
#[path="expected_float.rs"] mod expected;
#[test]
fn existing_float32_hull_is_unchanged(){
 let p:Vec<_>=input::INPUT_BITS.chunks_exact(3).map(|p|parry3d::math::Vector::new(f32::from_bits(p[0]),f32::from_bits(p[1]),f32::from_bits(p[2]))).collect();let(v,i)=parry3d::transformation::try_convex_hull(&p).unwrap();
 assert_eq!(v.iter().flat_map(|p|[p.x.to_bits(),p.y.to_bits(),p.z.to_bits()]).collect::<Vec<_>>(),expected::POSITIONS);
 assert_eq!(i.iter().flatten().copied().collect::<Vec<_>>(),expected::INDICES);
}
