// Research-only high-precision hull of unchanged stored Float32 points.
use parry3d_f64::{math::Vector,transformation::try_convex_hull};
fn main(){
 let path=std::env::args().nth(1).expect("Input bit file required");
 let words:Vec<u32>=std::fs::read_to_string(path).unwrap().split_whitespace().map(|s|s.parse().unwrap()).collect();
 assert!(words.len()>=12&&words.len()<=30000&&words.len()%3==0);
 let points:Vec<_>=words.chunks_exact(3).map(|p|Vector::new(f32::from_bits(p[0]) as f64,f32::from_bits(p[1]) as f64,f32::from_bits(p[2]) as f64)).collect();
 assert!(points.iter().all(|p|p.is_finite()));
 let (v,i)=try_convex_hull(&points).expect("Double hull rejected");
 assert!(v.iter().all(|p|points.contains(p)));
 print!("{{\"positions\":[");for(j,x)in v.iter().flat_map(|p|[p.x,p.y,p.z]).enumerate(){if j>0{print!(",");}print!("{:?}",x);}
 print!("],\"indices\":[");for(j,x)in i.iter().flatten().enumerate(){if j>0{print!(",");}print!("{}",x);}println!("]}}");
}
