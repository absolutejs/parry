//! Private diagnostic candidate: fixed-plane advancement with bounded curvature.
//! Not linked into the library or a released engine.
use parry3d::{math::{Real, Rotation, Vector}, query::sweep_toi::{proxy_distance, SimplexCache, Sweep, SweepToiOutput, SweepToiStatus, ToiProxy}};

fn endpoints(s:&Sweep)->(Rotation,Rotation) {
 let a=if s.q1.dot(s.q2)<0.0{-s.q1}else{s.q1}; (a,s.q2)
}
fn curvature(s:&Sweep,p:&ToiProxy)->Real {
 let(a,b)=endpoints(s);let delta=b-a;
 let min_norm_sq=(a+b).length_squared()*0.25;
 let radius=p.points().iter().map(|v|(*v-s.local_center).length()).fold(0.0_f32,Real::max);
 // u=q0+t*d, q=u/|u|: |q'|<=|d|/m, |q''|<=6|d|²/m².
 // R(q)v=q*v*q*: |(R(q)v)''|<=14|v||d|²/m².
 (14.0*radius*delta.length_squared()/min_norm_sq).next_up()
}
fn angular_velocity(s:&Sweep,t:Real)->Vector {
 let(a,b)=endpoints(s);let d=b-a;let u=a+d*t;let norm=u.length();let q=u/norm;
 let dq=(d-q*q.dot(d))/norm;let w=dq*q.conjugate();
 Vector::new(w.x,w.y,w.z)*2.0
}
fn projected(s:&Sweep,p:&ToiProxy,t:Real,n:Vector)->(Real,Real,Real,Real,Real) {
 let pose=s.transform_at(t);let omega=angular_velocity(s,t);let linear=s.c2-s.c1;
 let(mut lo,mut hi,mut vlo,mut vhi,mut magnitude)=(Real::MAX,-Real::MAX,Real::MAX,-Real::MAX,0.0_f32);
 for point in p.points() {
  let world=pose.transform_point(*point);let projection=world.dot(n);
  let arm=pose.rotation*(*point-s.local_center);
  let velocity=(linear+omega.cross(arm)).dot(n);
  lo=lo.min(projection);hi=hi.max(projection);vlo=vlo.min(velocity);vhi=vhi.max(velocity);
  magnitude=magnitude.max(world.abs().max_element());
 }
 (lo,hi,vlo,vhi,magnitude)
}
pub fn candidate(a:&ToiProxy,sa:&Sweep,b:&ToiProxy,sb:&Sweep,max_fraction:Real,slop:Real)->(SweepToiOutput,u32) {
 let origin=sa.c1;let sa=sa.shifted(origin);let sb=sb.shifted(origin);
 let target=slop.max(a.radius+b.radius-slop);let tolerance=0.25*slop;
 let bound=curvature(&sa,a)+curvature(&sb,b);let mut cache=SimplexCache::default();let mut t=0.0;
 let mut out=SweepToiOutput{status:SweepToiStatus::Failed,fraction:0.0,point:Vector::ZERO,normal:Vector::ZERO};
 for iteration in 1..=25 {
  let xa=sa.transform_at(t);let xb=sb.transform_at(t);
  let distance=proxy_distance(&xa.inv_mul(&xb),a,b,false,&mut cache);
  let pa=xa.transform_point(distance.point_a);let pb=xa.transform_point(distance.point_b);
  let witness_n=(pb-pa).normalize_or_zero();
  let gjk_n=xa.rotation*distance.normal;
  let plane_gap=|n:Vector|{let(_,amax,_,_,_)=projected(&sa,a,t,n);let(bmin,_,_,_,_)=projected(&sb,b,t,n);bmin-amax};
  let n=if plane_gap(gjk_n)>plane_gap(witness_n){gjk_n}else{witness_n};
  out.fraction=t;out.point=(pa+pb+(a.radius-b.radius)*n)*0.5+origin;out.normal=n;
  if distance.distance<=0.0 {out.status=if t==0.0{SweepToiStatus::Overlapped}else{SweepToiStatus::Failed};return(out,iteration)}
  if distance.distance<=target+tolerance {out.status=SweepToiStatus::Hit;return(out,iteration)}
  if t==max_fraction {out.status=SweepToiStatus::Separated;return(out,iteration)}
  if n==Vector::ZERO{return(out,iteration)}
  let(_,amax,_,avmax,amag)=projected(&sa,a,t,n);
  let(bmin,_,bvmin,_,bmag)=projected(&sb,b,t,n);
  let roundoff=64.0*Real::EPSILON*(amag+bmag+target);
  let margin=bmin-amax-target-roundoff;
  if margin<=0.0{return(out,iteration)}
  let closing=(avmax-bvmin).max(0.0);
  let denom=closing+(closing*closing+2.0*bound*margin).sqrt();
  let safe=if denom>0.0{2.0*margin/denom}else{Real::INFINITY};
  if safe>max_fraction-t {out.status=SweepToiStatus::Separated;out.fraction=max_fraction;return(out,iteration)}
  let next=(t+0.9*safe).min(max_fraction);
  if !next.is_finite()||next<=t{return(out,iteration)}t=next;
 }
 (out,25)
}
