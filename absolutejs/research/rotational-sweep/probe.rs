use parry3d::{math::{Pose, Rotation, Vector}, query::sweep_toi::{sweep_time_of_impact, Sweep, ToiProxy}, shape::{Cuboid, Shape, Triangle}};
fn main() {
 let mut rows=Vec::new();
 for scale in [0.01_f32,1.0,100.0] {for reverse in [false,true] {for clip in [0.1_f32,1.0] {
  let triangle=Triangle::new(Vector::new(-10.0,0.0,-10.0)*scale,Vector::new(0.0,0.0,10.0)*scale,Vector::new(10.0,0.0,-10.0)*scale);
  let bar=Cuboid::new(Vector::new(2.0,0.001,0.01)*scale);
  let start=Pose::from_parts(Vector::new(0.0,1.999,0.0)*scale,Rotation::from_rotation_z(87.0_f32.to_radians()));
  let end=Pose::from_parts(start.translation,Rotation::from_rotation_z(93.0_f32.to_radians()));
  let moving=Sweep::from_poses(&start,&end,Vector::ZERO);
  let fixed=Sweep::constant(&Pose::IDENTITY,Vector::ZERO);
  let a=ToiProxy::from_shape(&triangle).unwrap(); let b=ToiProxy::from_shape(&bar).unwrap();
  let result=if reverse{sweep_time_of_impact(&b,&moving,&a,&fixed,clip,0.00001*scale)}else{sweep_time_of_impact(&a,&fixed,&b,&moving,clip,0.00001*scale)};
  let start_gap=bar.compute_aabb(&start).mins.y/scale;
  let end_gap=bar.compute_aabb(&end).mins.y/scale;
  let middle_gap=bar.compute_aabb(&moving.transform_at(0.5)).mins.y/scale;
  assert!(start_gap>0.0&&end_gap>0.0&&middle_gap<0.0);
  rows.push(serde_json::json!({"scale":scale,"reverse":reverse,"clip":clip,"status":format!("{:?}",result.status),"fraction":result.fraction,"startPlaneGap":start_gap,"endPlaneGap":end_gap,"middlePlaneGap":middle_gap,"expected":if clip==1.0{"Hit"}else{"Separated"}}));
 }}}
 println!("{}",serde_json::to_string_pretty(&serde_json::json!({"scope":"Direct sweep, stationary triangle, rotating thin box; actual endpoint-interpolated motion; no composite traversal","rows":rows})).unwrap());
}
