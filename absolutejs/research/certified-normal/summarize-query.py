"""Summarize measured query consistency; this is not an exact-distance proof."""
from pathlib import Path
import json,math,hashlib
root=Path(__file__).parent; raw=(root/'query-report.json').read_bytes(); data=json.loads(raw);out=[]
for row in data['rows']:
    a,b=row['normal'],row['triangleNormal']
    denom=math.sqrt(sum(v*v for v in a)*sum(v*v for v in b))
    assert denom>0
    cosine=sum(x*y for x,y in zip(a,b))/denom
    out.append({'state':row['state'],'faceNormalAngleDegrees':math.degrees(math.acos(max(-1,min(1,cosine)))),'minimumReportedDistanceMillimetres':10*min(v['distance'] for v in row['points']),'maximumWitnessProjectionMismatchMillimetres':10*max(abs(v['witnessProjectedDistance']-v['distance']) for v in row['points']),'supportPlaneGapMillimetres':10*row['supportPlaneGapCentimetres']})
(root/'query-report-summary.json').write_text(json.dumps({'scope':__doc__+' Touching retains the existing scaled tolerance and can have a negative support gap. Angles normalize both recorded f32 vectors in f64.','inputSha256':hashlib.sha256(raw).hexdigest(),'rows':out},indent=2)+'\n')
print(out)
