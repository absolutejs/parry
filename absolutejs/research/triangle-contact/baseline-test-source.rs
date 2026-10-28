use parry3d::{math::{Pose,Vector,Rotation},query::{ContactManifold,ContactManifoldsWorkspace,DefaultQueryDispatcher,PersistentQueryDispatcher},shape::{ConvexPolyhedron,HeightField}};
use serde_json::Value;
fn fixture(state: &str)->(HeightField,ConvexPolyhedron,Pose,f32,u32) {
 let text=match state {"before"=>include_str!("fixtures/before.json"),_=>include_str!("fixtures/after.json")};
 let v:Value=serde_json::from_str(text).unwrap();
 let t:[f32;3]=serde_json::from_value(v["relativeTranslation"].clone()).unwrap();
 let q:[f32;4]=serde_json::from_value(v["relativeRotation"].clone()).unwrap();
 (serde_json::from_value(v["heightfield"].clone()).unwrap(),serde_json::from_value(v["convexShape"].clone()).unwrap(),Pose::from_parts(Vector::from_array(t),Rotation::from_xyzw(q[0],q[1],q[2],q[3])),v["prediction"].as_f64().unwrap() as f32,v["triangleId"].as_u64().unwrap() as u32)
}
fn check(state:&str,flipped:bool) {
 let (h,c,p,prediction,id)=fixture(state);
 let mut manifolds:Vec<ContactManifold<(),u32>>=Vec::new();
 let mut workspace:Option<ContactManifoldsWorkspace>=None;
 for iteration in 0..3 {
  if flipped {DefaultQueryDispatcher.contact_manifolds(&p.inverse(),&c,&h,prediction,&mut manifolds,&mut workspace).unwrap();}
  else {DefaultQueryDispatcher.contact_manifolds(&p,&h,&c,prediction,&mut manifolds,&mut workspace).unwrap();}
  let m=manifolds.iter().find(|m|if flipped{m.subshape2==id}else{m.subshape1==id}).unwrap();
  assert!(!m.points.is_empty());
  let minimum=m.points.iter().map(|p|p.dist).reduce(f32::min).unwrap();
  let limit=if state=="before"{0.001}else{0.02};
  assert!(minimum>=-limit,"{state} flipped={flipped} iteration={iteration}: {minimum} centimetres");
  assert!(m.local_n1.is_finite() && m.local_n2.is_finite());
  // Restore both the cache and its exact contact frame, then issue another query.
  let bytes=bincode::serialize(&(manifolds,workspace)).unwrap();
  (manifolds,workspace)=bincode::deserialize(&bytes).unwrap();
  assert_eq!(bytes,bincode::serialize(&(&manifolds,&workspace)).unwrap());
 }
}
#[test] fn before_heightfield_first(){check("before",false);}
#[test] fn before_heightfield_second(){check("before",true);}
#[test] fn after_heightfield_first(){check("after",false);}
#[test] fn after_heightfield_second(){check("after",true);}
