use parry3d::{math::{Pose,Rotation,Vector},query::{ContactManifold,DefaultQueryDispatcher,PersistentQueryDispatcher},shape::{ConvexPolyhedron,Triangle}};
use serde_json::Value;
#[test]
fn cached_approach_keeps_the_near_triangle_contact(){
 let v:Value=serde_json::from_str(include_str!("fixture.json")).unwrap();
 let vector=|v:&Value|Vector::from_array(serde_json::from_value::<[f32;3]>(v.clone()).unwrap());
 let q:[f32;4]=serde_json::from_value(v["relativeRotation"].clone()).unwrap();
 let p=Pose::from_parts(vector(&v["relativeTranslation"]),Rotation::from_xyzw(q[0],q[1],q[2],q[3]));
 let t=Triangle::new(vector(&v["triangle"][0]),vector(&v["triangle"][1]),vector(&v["triangle"][2]));
 let c:ConvexPolyhedron=serde_json::from_value(v["convexShape"].clone()).unwrap();
 let mut m:ContactManifold<(),()>=ContactManifold::new();m.local_n1=vector(&v["previousCached"]["normal1"]);
 DefaultQueryDispatcher.contact_manifold_convex_convex(&p,&t,&c,None,None,2.0,&mut m).unwrap();
 eprintln!("normal={:?} points={:?}",m.local_n1,m.points.iter().map(|p|p.dist).collect::<Vec<_>>());
 assert!(!m.points.is_empty(),"near triangle contact was lost");
 let nearest=m.points.iter().map(|p|p.dist).fold(f32::INFINITY,f32::min);
 assert!((nearest-0.01).abs()<0.003,"expected about 0.01 cm positive separation, got {nearest}");
}
#[test]
fn approach_scale_order_origin_and_clear_controls(){
 let v:Value=serde_json::from_str(include_str!("fixture.json")).unwrap();
 let vector=|v:&Value|Vector::from_array(serde_json::from_value::<[f32;3]>(v.clone()).unwrap());
 let q:[f32;4]=serde_json::from_value(v["relativeRotation"].clone()).unwrap();
 let original:ConvexPolyhedron=serde_json::from_value(v["convexShape"].clone()).unwrap();
 for scale in [0.01_f32,1.0,100.0] {for recenter in [false,true] {for flipped in [false,true] {for clear in [false,true] {
  let t=Triangle::new(vector(&v["triangle"][0])*scale,vector(&v["triangle"][1])*scale,vector(&v["triangle"][2])*scale);
  let c=original.clone().scaled(Vector::splat(scale)).unwrap();
  let origin=if recenter{t.a}else{Vector::ZERO};
  let mut p=Pose::from_parts(vector(&v["relativeTranslation"])*scale-origin,Rotation::from_xyzw(q[0],q[1],q[2],q[3]));
  let mut n=(t.b-t.a).cross(t.c-t.a).normalize();if n.y<0.0{n=-n;}
  if clear{p.translation+=n*(10.0*scale);}
  let triangle=Triangle::new(t.a-origin,t.b-origin,t.c-origin);
  let mut m:ContactManifold<(),()>=ContactManifold::new();m.local_n1=vector(&v["previousCached"][if flipped{"normal2"}else{"normal1"}]);
  for iteration in 0..3 {
   if flipped {DefaultQueryDispatcher.contact_manifold_convex_convex(&p.inverse(),&c,&triangle,None,None,2.0*scale,&mut m).unwrap();}
   else {DefaultQueryDispatcher.contact_manifold_convex_convex(&p,&triangle,&c,None,None,2.0*scale,&mut m).unwrap();}
   if clear {assert!(m.points.is_empty(),"clear control scale={scale} origin={recenter} flipped={flipped} iteration={iteration}");}
   else {
    assert!(!m.points.is_empty(),"lost contact scale={scale} origin={recenter} flipped={flipped} iteration={iteration}");
    let nearest=m.points.iter().map(|p|p.dist).fold(f32::INFINITY,f32::min)/scale;
    assert!((nearest-0.01).abs()<0.003,"bad distance {nearest} scale={scale} origin={recenter} flipped={flipped} iteration={iteration}");
   }
  }
 }}}}
}
