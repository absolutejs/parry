use parry3d::{math::{Pose, Rotation, Vector}, query::{ContactManifold, DefaultQueryDispatcher, PersistentQueryDispatcher}, shape::{ConvexPolyhedron, Triangle}};
use serde_json::Value;
#[test]
fn captured_oak_cached_direction_respects_triangle_plane_bound() {
 let v:Value=serde_json::from_str(include_str!("oak-fixture.json")).unwrap();
 let vec=|v:&Value|Vector::from_array(serde_json::from_value::<[f32;3]>(v.clone()).unwrap());
 let q:[f32;4]=serde_json::from_value(v["relativeRotation"].clone()).unwrap();
 let p=Pose::from_parts(vec(&v["relativeTranslation"]),Rotation::from_xyzw(q[0],q[1],q[2],q[3]));
 let t=Triangle::new(vec(&v["triangle"][0]),vec(&v["triangle"][1]),vec(&v["triangle"][2]));
 let c:ConvexPolyhedron=serde_json::from_value(v["convexShape"].clone()).unwrap();
 for recenter in [false,true] {
  let origin=if recenter {t.a}else{Vector::ZERO};
  let triangle=Triangle::new(t.a-origin,t.b-origin,t.c-origin);
  let mut pose=p;pose.translation-=origin;
  let mut m:ContactManifold<(),()>=ContactManifold::new();
  m.local_n1=vec(&v["cached"]["normal1"]);
  DefaultQueryDispatcher.contact_manifold_convex_convex(&pose,&triangle,&c,None,None,2.0,&mut m).unwrap();
  let depth=m.points.iter().map(|p|p.dist).fold(0.0,f32::min);
  eprintln!("recenter={recenter} depth_cm={depth} normal={:?}",m.local_n1);
  // Independent f64 plane bound is 0.001265 cm. Allow 0.001735 cm
  // for f32 projection; this is a regression assertion, no runtime tolerance change.
  assert!(depth>=-0.003,"false depth {depth} cm, recenter={recenter}");
 }
}
