"""Run exact library source under its selected runtime features without GUI examples.
The upstream workspace resolves unavailable optional/example dependencies before
--lib. This isolated test-only manifest retains the enabled runtime dependency
closure and only the library tests' used dev dependency (oorandom).
"""
from pathlib import Path
import tomllib,json,hashlib
root=Path(__file__).resolve().parents[3]
workspace=tomllib.loads((root/'Cargo.toml').read_text())['workspace']
crate=tomllib.loads((root/'crates/parry3d/Cargo.toml').read_text())
features=crate['features']; enabled=set(); active=set(); pending=['required-features','std','alloc','serde-serialize']
while pending:
 f=pending.pop()
 if f in enabled: continue
 if f not in features: active.add(f.removeprefix('dep:'));continue
 enabled.add(f)
 for item in features[f]:
  if '/' in item:
   dep=item.split('/')[0]
   if not dep.endswith('?'):active.add(dep)
  else:pending.append(item)
deps={}
for name,d in crate['dependencies'].items():
 if d.get('optional',False) and name not in active:continue
 inherited=workspace['dependencies'][name];v={'version':inherited} if isinstance(inherited,str) else dict(inherited)
 for k,x in d.items():
  if k=='workspace':continue
  if k=='features':v[k]=list(dict.fromkeys(v.get(k,[])+x))
  else:v[k]=x
 deps[name]=v
selected={k:[x for x in v if '/' not in x or x.split('/')[0].removesuffix('?') in deps] if k in enabled else [] for k,v in features.items()}
selected.update({'dim2':[], 'f64':[]})
for key in ['std','alloc','serde-serialize','required-features']:assert key in enabled
out={'workspace':{},'package':{'name':'parry3d','version':workspace['package']['version'],'edition':workspace['package']['edition'],'publish':False},'lib':{'name':'parry3d','path':'../../../../src/lib.rs'},'features':selected,'dependencies':deps,'dev-dependencies':{'oorandom':workspace['dependencies']['oorandom']}}
def value(v):
 if isinstance(v,dict):return '{'+','.join(json.dumps(k)+'='+value(x) for k,x in v.items())+'}'
 if isinstance(v,list):return '['+','.join(value(x) for x in v)+']'
 return json.dumps(v)
folder=Path(__file__).parent/'library';folder.mkdir(exist_ok=True)
text='\n'.join('['+table+']\n'+'\n'.join(json.dumps(k)+'='+value(v) for k,v in data.items()) for table,data in out.items())+'\n'
assert tomllib.loads(text)==out
(folder/'Cargo.toml').write_text(text)
(folder/'manifest-evidence.json').write_text(json.dumps({'scope':__doc__,'enabledFeatures':sorted(enabled),'runtimeDependencies':sorted(deps),'librarySource':'src/lib.rs','sourceFiles':{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((root/'src').rglob('*.rs'))},'manifestSha256':hashlib.sha256(text.encode()).hexdigest()},indent=2)+'\n')
