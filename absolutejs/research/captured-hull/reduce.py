"""Bounded greedy reduction retaining an exact strict export crossing.
Does not claim globally smallest input or preservation of original negative volume.
"""
from predicates import *
import subprocess,time,struct,gzip
root=Path(__file__).parent;out=root/'reduction'
if out.exists():raise ValueError('Evidence exists')
out.mkdir();binary=root/'target/release/absolutejs-captured-hull';binary_sha=hashlib.sha256(binary.read_bytes()).hexdigest()
source=json.loads((root/'baseline.json').read_text())
# Original Float32 words are stored independently of text-rounding conventions.
bits=[int(x.strip()) for x in (root/'src/input.rs').read_text().split('[\n',1)[1].split('];',1)[0].split(',') if x.strip()];points=[bits[i:i+3] for i in range(0,len(bits),3)]
ids=list(range(len(points)));attempts=[];start=time.monotonic();serial=0

def evaluate(indices):
 global serial
 serial+=1;path=out/'trial-input.txt';path.write_text('\n'.join(str(x) for i in indices for x in points[i])+'\n')
 proc=subprocess.run([str(binary.resolve()),str(path.resolve())],capture_output=True,text=True,timeout=3)
 if proc.returncode:return None,{'error':'native-rejected','stderr':proc.stderr[:1500]}
 data=json.loads(proc.stdout);m=data['export'];v=[tuple(map(F,m['positions'][i:i+3])) for i in range(0,len(m['positions']),3)];tris=[tuple(v[k] for k in m['indices'][i:i+3]) for i in range(0,len(m['indices']),3)];planes=[plane(t) for t in tris]
 for i in range(len(tris)):
  for j in range(i):
   if crossing(tris[i],tris[j],planes[i],planes[j]):return data,{'crossing':[i,j]}
 return None,{'crossing':None}

initial,detail=evaluate(ids);assert initial is not None
(out/'accepted-000.json.gz').write_bytes(gzip.compress(json.dumps(initial).encode()))
changed=True
while changed and len(ids)>4 and serial<300 and time.monotonic()-start<120:
 changed=False
 for removed in list(ids):
  if serial>=300 or time.monotonic()-start>=120:break
  candidate=[i for i in ids if i!=removed]
  data,detail=evaluate(candidate);accepted=data is not None
  row={'attempt':serial,'removedOriginalIndex':removed,'remaining':len(candidate),'accepted':accepted,**detail};attempts.append(row)
  with (out/'attempts.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
  if accepted:
   ids=candidate;changed=True;(out/f'accepted-{serial:03}.json.gz').write_bytes(gzip.compress(json.dumps(data).encode()));(out/'latest-input.txt').write_text('\n'.join(str(x) for i in ids for x in points[i])+'\n')
   print(json.dumps(row),flush=True)
   break
final,detail=evaluate(ids);assert final is not None
(out/'final.json').write_text(json.dumps(final,indent=2)+'\n');(out/'final-input.txt').write_text('\n'.join(str(x) for i in ids for x in points[i])+'\n')
assert hashlib.sha256(binary.read_bytes()).hexdigest()==binary_sha
report={'scope':'Greedy deletion preserving strict native-export crossing; not original Boolean or globally minimal input','sourceNativePin':'53ac0b0b02767300da09d092d97cef53a975c947','binarySha256':binary_sha,'scriptSha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'inputPoints':len(points),'finalPoints':len(ids),'retainedOriginalIndices':ids,'attempts':serial,'completedSingleDeletionSweep':not changed,'elapsedSeconds':time.monotonic()-start,'finalCrossing':detail}
(out/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
