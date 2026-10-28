mod interval_candidate;
mod math {pub use parry3d::math::*;}
use math::{Real,Vector};
use parry3d::query::sweep_toi::{Sweep,SweepToiOutput,SweepToiStatus,ToiProxy};
fn sweep_time_of_impact(a:&ToiProxy,sa:&Sweep,b:&ToiProxy,sb:&Sweep,max:Real,slop:Real)->SweepToiOutput {
 interval_candidate::candidate(a,sa,b,sb,max,slop).0
}
include!("upstream-controls.rs");
