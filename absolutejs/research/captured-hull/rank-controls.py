"""Certify thin captured clouds and deterministic transforms without mesh repair."""
from predicates import *
import subprocess,struct,gzip,time
root=Path(__file__).parent;out=root/'rank-controls'
if out.exists():raise ValueError('Evidence exists')
out.mkdir();binary=root/'target/release/export_control';sha=hashlib.sha256(binary.read_bytes()).hexdigest();rows=[];start=time.monotonic()
def values(path):
 words=list(map(int,path.read_text().split()));return [struct.unpack('<f',struct.pack('<I',v))[0] for v in words]
def audit(mesh,input_values,planar):
 vertices=[tuple(map(F,mesh['positions'][i:i+3])) for i in range(0,len(mesh['positions']),3)];points=[tuple(map(F,input_values[i:i+3])) for i in range(0,len(input_values),3)];edges={};vol=F(0);violations=0;degenerate=0
 assert all(p in points for p in vertices)
 for i in range(0,len(mesh['indices']),3):
  ids=mesh['indices'][i:i+3];a,b,c=[vertices[k] for k in ids];n,d=plane([a,b,c]);degenerate+=not any(n);violations+=sum(dot(n,p)>d for p in points);vol+=dot(a,cross(b,c))/6
  for k in range(3):key=(ids[k],ids[(k+1)%3]);edges[key]=edges.get(key,0)+1
 paired=all(count==1 and edges.get((b,a))==1 for (a,b),count in edges.items())
 return {'volume':float(vol),'positivePlaneTests':violations,'degenerateTriangles':degenerate,'pairedEdges':paired,'accepted':not violations and not degenerate and paired and (vol==0 if planar else vol>0)}
cases=[]
for path in sorted((root/'rank-fixtures').glob('input-*.txt')):
 v=values(path);points=[v[i:i+3] for i in range(0,len(v),3)]
 for scale in [2**-8,1,2**8]:
  for rotation in [0,1,2]:
   for reverse in [False,True]:
    ordered=points[::-1] if reverse else points
    flat=[p[(k+rotation)%3]*scale for p in ordered for k in range(3)]
    cases.append((f'{path.stem}-{scale}-{rotation}-{int(reverse)}',flat,False))
for axis in range(3):
 for height in [0,2**-20,2**-30]:
  p=[[0,0,0],[1,0,0],[0,1,0],[.25,.25,height]] if height else [[0,0,0],[1,0,0],[0,1,0],[1,1,0]]
  cases.append((f'plane-axis-{axis}-height-{height}',[v[(k+axis)%3] for v in p for k in range(3)],height==0))
for key,flat,planar in cases:
 if time.monotonic()-start>60:raise ValueError('60 second budget')
 words=[struct.unpack('<I',struct.pack('<f',v))[0] for v in flat];path=out/'input.txt';path.write_text('\n'.join(map(str,words))+'\n')
 proc=subprocess.run([str(binary.resolve()),str(path.resolve())],capture_output=True,text=True,timeout=3)
 if proc.returncode:row={'key':key,'accepted':False,'error':proc.stderr[:1000]}
 else:
  mesh=json.loads(proc.stdout);row={'key':key,**audit(mesh,flat,planar)}
  (out/(key+'.json.gz')).write_bytes(gzip.compress(json.dumps({'input':flat,'planar':planar,'mesh':mesh,'audit':row}).encode()))
 rows.append(row)
 with (out/'progress.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
assert hashlib.sha256(binary.read_bytes()).hexdigest()==sha
report={'scope':'Exact rational input containment, paired oriented edges and volume for captured thin clouds and deterministic scale/axis/order transforms','cases':len(rows),'accepted':sum(r['accepted'] for r in rows),'rejected':[r for r in rows if not r['accepted']],'binarySha256':sha,'scriptSha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'elapsedSeconds':time.monotonic()-start}
(out/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));raise SystemExit(0 if all(r['accepted'] for r in rows) else 1)
