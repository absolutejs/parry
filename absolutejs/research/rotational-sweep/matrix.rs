mod interval_candidate;
use glamx::{DQuat,DVec3};
use parry3d::{math::{Pose,Rotation,Vector},query::sweep_toi::{sweep_time_of_impact,Sweep,ToiProxy},shape::{Cuboid,Triangle}};
fn dv(v:Vector)->DVec3{DVec3::new(v.x as f64,v.y as f64,v.z as f64)}
fn dq(q:Rotation)->DQuat{DQuat::from_xyzw(q.x as f64,q.y as f64,q.z as f64,q.w as f64)}
// Independent explicit corner/plane oracle. The grid brackets a crossing for
// measurement; it is not the production interval certificate.
fn gap(s:&Sweep,points:&[Vector],floor:&Pose,t:f64)->f64 {
 let mut a=dq(s.q1);let b=dq(s.q2);if a.dot(b)<0.0{a=-a};let q=(a*(1.0-t)+b*t).normalize();
 let center=dv(s.c1)*(1.0-t)+dv(s.c2)*t;
 let n=dq(floor.rotation).normalize()*DVec3::Y;let origin=dv(floor.translation);
 points.iter().map(|p|(center+q*(dv(*p)-dv(s.local_center))-origin).dot(n)).fold(f64::INFINITY,f64::min)
}
fn first_crossing(s:&Sweep,points:&[Vector],floor:&Pose,clip:f64,target:f64)->Option<f64> {
 if gap(s,points,floor,0.0)<=target{return Some(0.0)}
 for i in 1..=2048 {let hi=clip*i as f64/2048.0;if gap(s,points,floor,hi)<=target {
  let(mut lo,mut hi)=(clip*(i-1)as f64/2048.0,hi);
  for _ in 0..45{let mid=(lo+hi)*0.5;if gap(s,points,floor,mid)>target{lo=mid}else{hi=mid}}
  return Some(hi)
 }}None
}
fn main(){
 let mut rows=Vec::new();
 for scale in [0.01_f32,1.0,100.0]{for start_angle in [87.0_f32,60.0,45.0]{for offset in [Vector::ZERO,Vector::new(0.3,0.2,0.1),Vector::new(-0.2,0.1,0.0)]{for rotated in [false,true]{for reverse in [false,true]{for clip in [0.1_f32,1.0]{for negate in [false,true]{
  let global=if rotated{Rotation::from_rotation_x(0.37)*Rotation::from_rotation_y(-0.21)}else{Rotation::IDENTITY};
  let floor=Pose::from_parts(Vector::ZERO,global);
  let triangle=Triangle::new(Vector::new(-10.0,0.0,-10.0)*scale,Vector::new(0.0,0.0,10.0)*scale,Vector::new(10.0,0.0,-10.0)*scale);
  let bar=Cuboid::new(Vector::new(2.0,0.001,0.01)*scale);
  let start=Pose::from_parts(global*(Vector::new(0.0,1.999,0.0)*scale),global*Rotation::from_rotation_z(start_angle.to_radians()));
  let end=Pose::from_parts(start.translation,global*Rotation::from_rotation_z((180.0-start_angle).to_radians()));
  let mut moving=Sweep::from_poses(&start,&end,offset*scale);if negate{moving.q1=-moving.q1}
  let fixed=Sweep::constant(&floor,Vector::ZERO);let a=ToiProxy::from_shape(&triangle).unwrap();let b=ToiProxy::from_shape(&bar).unwrap();let slop=0.00001*scale;
  let (candidate,iterations)=if reverse{interval_candidate::candidate(&b,&moving,&a,&fixed,clip,slop)}else{interval_candidate::candidate(&a,&fixed,&b,&moving,clip,slop)};
  let library=if reverse{sweep_time_of_impact(&b,&moving,&a,&fixed,clip,slop)}else{sweep_time_of_impact(&a,&fixed,&b,&moving,clip,slop)};
  let expected=first_crossing(&moving,b.points(),&floor,clip as f64,slop as f64);
  let actual_gap=gap(&moving,b.points(),&floor,candidate.fraction as f64)/scale as f64;
  rows.push(serde_json::json!({"scale":scale,"startAngle":start_angle,"offset":[offset.x,offset.y,offset.z],"rotated":rotated,"reverse":reverse,"clip":clip,"negatedQuaternion":negate,"libraryStatus":format!("{:?}",library.status),"libraryFraction":library.fraction,"candidateStatus":format!("{:?}",candidate.status),"candidateFraction":candidate.fraction,"iterations":iterations,"planeGapAtCandidatePerScale":actual_gap,"oracleTargetFraction":expected}));
 }}}}}}}
 for row in &rows {
  assert_eq!(row["libraryStatus"],row["candidateStatus"],"Native routing differs");
  assert_eq!(row["libraryFraction"],row["candidateFraction"],"Native fraction differs");
  let expected=row["oracleTargetFraction"].as_f64();let status=row["candidateStatus"].as_str().unwrap();
  assert_eq!(status,if expected.is_some(){"Hit"}else{"Separated"},"{row}");
  assert!(row["iterations"].as_u64().unwrap()<=25);
  if let Some(first)=expected {
   let gap=row["planeGapAtCandidatePerScale"].as_f64().unwrap();
   assert!(gap>=0.00001-2.0*f32::EPSILON as f64&&gap<=0.0000125+2.0*f32::EPSILON as f64,"{row}");
   assert!(row["candidateFraction"].as_f64().unwrap()<=first+1e-4,"Later root: {row}");
  }
 }
 for pair in rows.chunks_exact(2) {
  assert_eq!(pair[0]["candidateStatus"],pair[1]["candidateStatus"]);
  assert_eq!(pair[0]["candidateFraction"],pair[1]["candidateFraction"],"Quaternion sign changed motion");
 }
 println!("{}",serde_json::to_string_pretty(&serde_json::json!({"scope":"Private candidate against corner-plane oracle; grid used only to bracket test oracle, not by candidate","rows":rows})).unwrap());
}
