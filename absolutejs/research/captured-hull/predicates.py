from fractions import Fraction as F
from pathlib import Path
import json,math,hashlib,sys

def sub(a,b):return tuple(x-y for x,y in zip(a,b))
def dot(a,b):return sum((x*y for x,y in zip(a,b)),F(0))
def cross(a,b):return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def plane(t):
 n=cross(sub(t[1],t[0]),sub(t[2],t[0]));return n,dot(n,t[0])
def crossing(a,b,pa,pb):
 na,da=pa;nb,db=pb;sa=[dot(nb,p)-db for p in a];sb=[dot(na,p)-da for p in b]
 if not(min(sa)<0<max(sa) and min(sb)<0<max(sb)):return False
 for tri,target,sides,n in [(a,b,sa,nb),(b,a,sb,na)]:
  for i in range(3):
   j=(i+1)%3
   if sides[i]*sides[j]>=0:continue
   t=sides[i]/(sides[i]-sides[j]);p=tuple(tri[i][k]+t*(tri[j][k]-tri[i][k]) for k in range(3))
   if all(dot(cross(sub(target[(k+1)%3],target[k]),sub(p,target[k])),n)>0 for k in range(3)):return True
 return False
