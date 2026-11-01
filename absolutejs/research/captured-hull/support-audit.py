"""Exact plane containment of every original input point, not only hull vertices."""
from predicates import *
import struct
root=Path(__file__).parent;source=root/sys.argv[1];out=source.with_name(source.stem+'-support.json')
if out.exists():raise ValueError('Evidence exists')
if len(sys.argv)>2:bits=list(map(int,(root/sys.argv[2]).read_text().split()))
else:bits=[int(x.strip()) for x in (root/'src/input.rs').read_text().split('[\n',1)[1].split('];',1)[0].split(',') if x.strip()]
v=[F(struct.unpack('<f',struct.pack('<I',x))[0]) for x in bits];inputs=[tuple(v[i:i+3]) for i in range(0,len(v),3)];data=json.loads(source.read_text());rows={}
for name,m in data.items():
 if not isinstance(m,dict) or 'positions' not in m:continue
 vertices=[tuple(map(F,m['positions'][i:i+3])) for i in range(0,len(m['positions']),3)];tris=[tuple(vertices[k] for k in m['indices'][i:i+3]) for i in range(0,len(m['indices']),3)];worst={'distanceCentimetres':0};count=0;degenerate=0;volume=F(0)
 for i,t in enumerate(tris):
  n,d=plane(t);norm=math.hypot(*map(float,n));volume+=dot(t[0],cross(t[1],t[2]))/6
  if not norm:degenerate+=1;continue
  for j,p in enumerate(inputs):
   side=dot(n,p)-d
   if side>0:
    count+=1;distance=float(side)/norm
    if distance>worst['distanceCentimetres']:worst={'triangle':i,'originalPoint':j,'distanceCentimetres':distance}
 rows[name]={'inputPointCount':len(inputs),'positiveExactPlaneTests':count,'worstInputPlaneViolation':worst,'degenerateTriangles':degenerate,'signedVolumeCubicCentimetres':float(volume),'allBoundaryVerticesAreOriginalInputs':all(p in inputs for p in vertices)}
report={'sourceSha256':hashlib.sha256(source.read_bytes()).hexdigest(),'scriptSha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'rows':rows}
out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(rows,indent=2))
