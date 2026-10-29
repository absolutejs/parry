"""Exact-decimal orientation of the captured binary32 simplex, independent of Parry."""
import hashlib,json,struct
from decimal import Decimal,localcontext
from pathlib import Path
root=Path(__file__).parent
log=(root/'point-trace.log').read_text()
lines=[line.split('TERRAIN_POINTS ',1)[1] for line in log.splitlines() if line.startswith('TERRAIN_POINTS ')]
assert len(lines)==1,lines
binary32=lambda x:struct.unpack('<f',struct.pack('<f',x))[0]
points=[[binary32(x) for x in p] for p in json.loads(lines[0])]
sub=lambda a,b:[x-y for x,y in zip(a,b)]
dot=lambda a,b:sum((x*y for x,y in zip(a,b)),Decimal(0))
cross=lambda a,b:[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
with localcontext() as ctx:
 ctx.prec=100
 p=[[Decimal.from_float(x) for x in a] for a in points]
 rows=[]
 for face,opposite in [([0,1,2],3),([0,1,3],2),([0,2,3],1),([1,2,3],0)]:
  a,b,c=[p[i] for i in face];n=cross(sub(b,a),sub(c,a));opp=dot(n,sub(p[opposite],a));origin=dot(n,[-x for x in a]);norm2=dot(n,n)
  rows.append({'face':face,'opposite':opposite,'oppositeOrientationExact':str(opp),'originOrientationExact':str(origin),'originStrictlyOutside':opp*origin<0,'originPlaneDistance':str(abs(origin)/norm2.sqrt()) if norm2 else None})
 result={'scope':__doc__,'sourceLogSha256':hashlib.sha256(log.encode()).hexdigest(),'binary32Points':points,'faces':rows,'outsideAtLeastOneFace':any(r['originStrictlyOutside'] for r in rows),'degenerateVolume':all(Decimal(r['oppositeOrientationExact'])==0 for r in rows)}
(root/'simplex-oracle.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
