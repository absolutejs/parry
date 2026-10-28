use parry3d::{math::{Pose,Vector,Rotation},query::{ContactManifold,DefaultQueryDispatcher,PersistentQueryDispatcher},shape::{ConvexPolyhedron,HeightField,SupportMap}};
use serde_json::{json,Value};
fn main(){
 let mut rows=Vec::new();
 for (name,text) in [("before",include_str!("fixtures/before.json")),("after",include_str!("fixtures/after.json"))]{
  let v:Value=serde_json::from_str(text).unwrap();
  let h:HeightField=serde_json::from_value(v["heightfield"].clone()).unwrap();
  let c:ConvexPolyhedron=serde_json::from_value(v["convexShape"].clone()).unwrap();
  let t:[f32;3]=serde_json::from_value(v["relativeTranslation"].clone()).unwrap();
  let q:[f32;4]=serde_json::from_value(v["relativeRotation"].clone()).unwrap();
  let p=Pose::from_parts(Vector::from_array(t),Rotation::from_xyzw(q[0],q[1],q[2],q[3]));
  let triangle=h.triangle_at_id(v["triangleId"].as_u64().unwrap() as u32).unwrap();
  let mut m:ContactManifold<(),()>=ContactManifold::new();
  DefaultQueryDispatcher.contact_manifold_convex_convex(&p,&triangle,&c,None,None,v["prediction"].as_f64().unwrap() as f32,&mut m).unwrap();
  let n=m.local_n1;
  let a=triangle.local_support_point(n);
  let b=p.transform_point(c.local_support_point(p.rotation.inverse()*-n));
  rows.push(json!({"state":name,"normal":n.to_array(),"triangleNormal":triangle.normal().unwrap().to_array(),"supportPlaneGapCentimetres":(b-a).dot(n),"points":m.points.iter().map(|pt|json!({"distance":pt.dist,"first":pt.local_p1.to_array(),"second":pt.local_p2.to_array(),"witnessProjectedDistance":(p.transform_point(pt.local_p2)-pt.local_p1).dot(n)})).collect::<Vec<_>>()}));
 }
 std::fs::write(std::env::args().nth(1).unwrap(),serde_json::to_string_pretty(&json!({"scope":"Unshifted standalone triangle-convex queries; support-gap and witness consistency report in solver centimetres", "rows":rows})).unwrap()+"\n").unwrap();
}
