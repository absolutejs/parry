mod interval_candidate;
use parry3d::{math::{Pose,Rotation,Vector},query::sweep_toi::{sweep_time_of_impact,Sweep,ToiProxy},shape::{Cuboid,Triangle}};
use std::{hint::black_box,time::Instant};
fn run(name:&str,a:&ToiProxy,sa:&Sweep,b:&ToiProxy,sb:&Sweep)->serde_json::Value {
 let baseline=sweep_time_of_impact(a,sa,b,sb,1.0,0.00001);let(result,queries)=interval_candidate::candidate(a,sa,b,sb,1.0,0.00001);
 let mut samples=Vec::new();
 for trial in 0..12 {
  let mut ms=[0.0;2];
  for engine in if trial%2==0{[0,1]}else{[1,0]} {
   let start=Instant::now();
   for _ in 0..1024 {
    let value=if engine==0{sweep_time_of_impact(black_box(a),black_box(sa),black_box(b),black_box(sb),1.0,0.00001)}else{interval_candidate::candidate(black_box(a),black_box(sa),black_box(b),black_box(sb),1.0,0.00001).0};
    black_box(value);
   }
   ms[engine]=start.elapsed().as_secs_f64()*1000.0;
  }
  samples.push(serde_json::json!({"trial":trial,"warmup":trial<4,"baselineMs":ms[0],"candidateMs":ms[1]}));
 }
 serde_json::json!({"scene":name,"queriesPerSample":1024,"baselineStatus":format!("{:?}",baseline.status),"baselineFraction":baseline.fraction,"candidateStatus":format!("{:?}",result.status),"candidateFraction":result.fraction,"candidateQueries":queries,"samples":samples})
}
fn main(){
 let wall=Cuboid::new(Vector::splat(0.5));let wall=ToiProxy::from_shape(&wall).unwrap();let fixed=Sweep::constant(&Pose::IDENTITY,Vector::ZERO);
 let translated=Sweep::from_poses(&Pose::translation(-4.0,0.0,0.0),&Pose::translation(4.0,0.0,0.0),Vector::ZERO);
 let missed=Sweep::from_poses(&Pose::translation(-4.0,5.0,0.0),&Pose::translation(4.0,5.0,0.0),Vector::ZERO);
 let bar=ToiProxy::from_array([Vector::new(-2.0,0.0,0.0),Vector::new(2.0,0.0,0.0)],0.05);
 let rotated=Sweep::from_poses(&Pose::IDENTITY,&Pose::from_parts(Vector::ZERO,Rotation::from_rotation_z(core::f32::consts::FRAC_PI_2)),Vector::ZERO);
 let ball=ToiProxy::point(Vector::ZERO,0.1);let d=1.5*0.5_f32.sqrt();let ball_pose=Sweep::constant(&Pose::translation(d,d,0.0),Vector::ZERO);
 let triangle=Triangle::new(Vector::new(-10.0,0.0,-10.0),Vector::new(0.0,0.0,10.0),Vector::new(10.0,0.0,-10.0));let floor=ToiProxy::from_shape(&triangle).unwrap();
 let thin=Cuboid::new(Vector::new(2.0,0.001,0.01));let thin=ToiProxy::from_shape(&thin).unwrap();
 let thin_sweep=|height|Sweep::from_poses(&Pose::from_parts(Vector::new(0.0,height,0.0),Rotation::from_rotation_z(87.0_f32.to_radians())),&Pose::from_parts(Vector::new(0.0,height,0.0),Rotation::from_rotation_z(93.0_f32.to_radians())),Vector::ZERO);
 let rows=vec![run("translation-hit",&wall,&fixed,&wall,&translated),run("translation-miss",&wall,&fixed,&wall,&missed),run("rotating-bar-ball",&bar,&rotated,&ball,&ball_pose),run("rotating-interior-crossing",&floor,&fixed,&thin,&thin_sweep(1.999)),run("rotating-clear",&floor,&fixed,&thin,&thin_sweep(3.0))];
 println!("{}",serde_json::to_string_pretty(&serde_json::json!({"scope":"Private candidate native CPU samples, alternating order, four warmups/eight measured pairs; not browser FPS; baseline misses interior-crossing fixture","rows":rows})).unwrap());
}
