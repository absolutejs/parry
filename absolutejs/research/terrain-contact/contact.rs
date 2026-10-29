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
