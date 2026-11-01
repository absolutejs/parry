mod input;
use parry3d::{math::Vector,shape::ConvexPolyhedron,transformation::try_convex_hull};
fn mesh(name:&str,vertices:&[Vector],indices:&[[u32;3]]){
 print!("\"{}\":{{\"positions\":[",name);
 for (i,v) in vertices.iter().flat_map(|v|[v.x,v.y,v.z]).enumerate(){if i>0{print!(",");} print!("{:?}",v as f64);}
 print!("],\"indices\":[");
 for(i,v) in indices.iter().flatten().enumerate(){if i>0{print!(",");}print!("{}",v);}
 print!("]}}");
}
fn main(){
 let bits:Vec<u32>=std::env::args().nth(1).map(|p|std::fs::read_to_string(p).unwrap().split_whitespace().map(|s|s.parse().unwrap()).collect()).unwrap_or_else(||input::INPUT_BITS.to_vec());
 assert!(bits.len()%3==0 && bits.len()>=12 && bits.len()<=30000);
 let points:Vec<_>=bits.chunks_exact(3).map(|p|Vector::new(f32::from_bits(p[0]),f32::from_bits(p[1]),f32::from_bits(p[2]))).collect();
 let (v,i)=try_convex_hull(&points).expect("Hull failed");
 let poly=ConvexPolyhedron::from_convex_mesh(v.clone(),&i).expect("Polyhedron failed");
 let (pv,pi)=poly.to_trimesh();
 let (ev,ei)=try_convex_hull(poly.points()).expect("Export hull failed");
 print!("{{");mesh("quickhull",&v,&i);print!(",");mesh("polyhedron",&pv,&pi);print!(",");mesh("export",&ev,&ei);
 print!(",\"faces\":[");
 for (j,f) in poly.faces().iter().enumerate(){if j>0{print!(",");}let start=f.first_vertex_or_edge as usize;let end=start+f.num_vertices_or_edges as usize;print!("{{\"vertices\":{:?},\"normal\":[{:?},{:?},{:?}]}}",&poly.vertices_adj_to_face()[start..end],f.normal.x as f64,f.normal.y as f64,f.normal.z as f64);}
 print!("],");
 let points64:Vec<_>=points.iter().map(|p|parry3d_f64::math::Vector::new(p.x as f64,p.y as f64,p.z as f64)).collect();
 let (dv,di)=parry3d_f64::transformation::try_convex_hull(&points64).expect("Double hull failed");
 print!("\"doubleQuickhull\":{{\"positions\":[");
 for(i,v)in dv.iter().flat_map(|v|[v.x,v.y,v.z]).enumerate(){if i>0{print!(",");}print!("{:?}",v);}
 print!("],\"indices\":[");for(i,v)in di.iter().flatten().enumerate(){if i>0{print!(",");}print!("{}",v);}println!("]}}}}");
}
