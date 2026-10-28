//! Private diagnostic candidate: fixed-plane advancement with bounded curvature.
//! Not linked into the library or a released engine.
use parry3d::{math::{Real, Vector}, query::sweep_toi::{proxy_distance, SimplexCache, Sweep, SweepToiOutput, SweepToiStatus, ToiProxy}};

use glamx::{DQuat,DVec3};
fn vec(v:Vector)->DVec3 { DVec3::new(v.x as f64,v.y as f64,v.z as f64) }
fn endpoints(s:&Sweep)->(DQuat,DQuat) {
 let a=DQuat::from_xyzw(s.q1.x as f64,s.q1.y as f64,s.q1.z as f64,s.q1.w as f64);
 let b=DQuat::from_xyzw(s.q2.x as f64,s.q2.y as f64,s.q2.z as f64,s.q2.w as f64);
 (if a.dot(b)<0.0{-a}else{a},b)
}
fn curvature(s:&Sweep,p:&ToiProxy)->f64 {
 let(a,b)=endpoints(s);let delta=b-a;let extremum=if delta.length_squared()>0.0{(-a.dot(delta)/delta.length_squared()).clamp(0.0,1.0)}else{0.0};
 let min_norm_sq=(a+delta*extremum).length_squared().next_down();
 let radius=p.points().iter().map(|v|(vec(*v)-vec(s.local_center)).length()).fold(0.0_f64,f64::max);
 // u=q0+t*d, q=u/|u|: |q'|<=|d|/m, |q''|<=3|d|²/m².
 // R(q)v=q*v*q*: |(R(q)v)''|<=8|v||d|²/m².
 (8.0*radius*delta.length_squared()/min_norm_sq).next_up()
}
fn projected_pair(s:&Sweep,p:&ToiProxy,t:Real,axes:[Vector;2])->[(f64,f64,f64,f64,f64);2] {
 let(a,b)=endpoints(s);let d=b-a;let u=a+d*(t as f64);let norm=u.length();let q=u/norm;
 let dq=(d-q*q.dot(d))/norm;let w=dq*q.conjugate();let omega=DVec3::new(w.x,w.y,w.z)*2.0;
 let linear=vec(s.c2)-vec(s.c1);let center=vec(s.c1)+linear*(t as f64);
 let axes=axes.map(|n|vec(n).normalize_or_zero());
 let mut values=[(f64::MAX,-f64::MAX,f64::MAX,-f64::MAX,0.0_f64);2];
 for point in p.points() {
  let arm=q*(vec(*point)-vec(s.local_center));let world=center+arm;
  let velocity=linear+omega.cross(arm);
  for (v,axis) in values.iter_mut().zip(axes) {
   let projection=world.dot(axis);let speed=velocity.dot(axis);
   v.0=v.0.min(projection);v.1=v.1.max(projection);v.2=v.2.min(speed);v.3=v.3.max(speed);v.4=v.4.max(world.abs().dot(axis.abs()));
  }
 }
 values
}
pub fn candidate(a:&ToiProxy,sa:&Sweep,b:&ToiProxy,sb:&Sweep,max_fraction:Real,slop:Real)->(SweepToiOutput,u32) {
 let constant=|s:&Sweep|s.q1==s.q2||s.q1== -s.q2;
 if constant(sa)&&constant(sb){return(parry3d::query::sweep_toi::sweep_time_of_impact(a,sa,b,sb,max_fraction,slop),0)}
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
  let axes=[witness_n,gjk_n];
  let ap=projected_pair(&sa,a,t,axes);let bp=projected_pair(&sb,b,t,axes);
  let selected=if bp[1].0-ap[1].1>bp[0].0-ap[0].1{1}else{0};let n=axes[selected];
  out.fraction=t;out.point=(pa+pb+(a.radius-b.radius)*n)*0.5+origin;out.normal=n;
  if distance.distance<=0.0 {out.status=if t==0.0{SweepToiStatus::Overlapped}else{SweepToiStatus::Failed};return(out,iteration)}
  if distance.distance<=target+tolerance {out.status=SweepToiStatus::Hit;return(out,iteration)}
  if t==max_fraction {out.status=SweepToiStatus::Separated;return(out,iteration)}
  if n==Vector::ZERO{return(out,iteration)}
  let(_,amax,_,avmax,amag)=ap[selected];
  let(bmin,_,bvmin,_,bmag)=bp[selected];
  let roundoff=64.0*f64::EPSILON*(amag+bmag+target as f64);
  let margin=bmin-amax-target as f64-roundoff;
  if margin<=0.0{return(out,iteration)}
  let closing=(avmax-bvmin).max(0.0);
  let denom=closing+(closing*closing+2.0*bound*margin).sqrt();
  let safe=if denom>0.0{2.0*margin/denom}else{f64::INFINITY};
  if safe>(max_fraction-t) as f64 {out.status=SweepToiStatus::Separated;out.fraction=max_fraction;return(out,iteration)}
  let next=((t as f64+0.9*safe).min(max_fraction as f64) as f32).next_down();
  if !next.is_finite()||next<=t{return(out,iteration)}t=next;
 }
 (out,25)
}
