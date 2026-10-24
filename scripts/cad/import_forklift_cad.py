from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
from scipy.spatial import cKDTree
import math, json, shutil, struct
R=Path(__file__).resolve().parents[2]; P=R/'src/forklift_description'; S=P/'cad_source'; orig=S/'forklift.original.urdf'
MESH_NAMES={'base_link': 'chassis', 'lidar_Link': 'lidar_housing', 'step_Link': 'lift_transmission', 'banhrang1_Link': 'lift_drive_gear', 'banhrang2_Link': 'lift_screw', 'can_Link': 'lift_carriage', 'banh1_Link': 'rear_right_wheel', 'banh2_Link': 'front_right_wheel', 'banh3_Link': 'front_left_wheel', 'banh4_Link': 'rear_left_wheel'}
(P/'meshes').mkdir(exist_ok=True)
root=E.parse(orig).getroot(); dt=np.dtype([('normal','<f4',3),('verts','<f4',(3,3)),('attr','<u2')])
def matrix(j):
 o=j.find('origin');xyz=list(map(float,o.get('xyz').split()));r,p,y=map(float,o.get('rpy').split());a,b,c=math.cos(r),math.cos(p),math.cos(y);d,e,f=math.sin(r),math.sin(p),math.sin(y)
 t=np.eye(4);t[:3,:3]=np.array([[c,-f,0],[f,c,0],[0,0,1]])@np.array([[b,0,e],[0,1,0],[-e,0,b]])@np.array([[1,0,0],[0,a,-d],[0,d,a]]);t[:3,3]=xyz;return t
T={'base_link':np.eye(4)}
for j in root.findall('joint'):T[j.find('child').get('link')]=T[j.find('parent').get('link')]@matrix(j)
meshes={l.get('name'):np.frombuffer((S/'meshes'/f"{l.get('name')}.STL").read_bytes(),dt,offset=84).copy() for l in root.findall('link')}
(R/'results/model_layout').mkdir(parents=True,exist_ok=True)
# Export contains duplicate child assemblies in base_link. Remove identical
# triangles only; no scaling, decimation or repositioning of CAD vertices.
base=meshes['base_link'];bv=base['verts'].astype(float);remove=np.zeros(len(base),bool);counts={}
for name,m in meshes.items():
 if name=='base_link':continue
 v=m['verts'].astype(float)@T[name][:3,:3].T+T[name][:3,3];tree=cKDTree(v.mean(axis=1));dist,idx=tree.query(bv.mean(axis=1));ids=np.where(dist<2e-5)[0];n=0
 for k in ids:
  a=bv[k];b=v[idx[k]];ds=np.linalg.norm(a[:,None,:]-b[None,:,:],axis=2)
  if max(ds.min(axis=0).max(),ds.min(axis=1).max())<2e-5:remove[k]=True;n+=1
 counts[name]=n
clean=base[~remove];(P/'meshes/chassis.stl').write_bytes(b'CAD duplicate child triangles removed; original dimensions preserved'.ljust(80,b' ')+struct.pack('<I',len(clean))+clean.tobytes())
# Ground root offset = nominal 50 mm radius minus CAD axle z (-20.4 mm).
offset=.0704
out=E.Element('robot',{'name':'forklift_cad','xmlns:xacro':'http://www.ros.org/wiki/xacro','xmlns:ignition':'http://ignitionrobotics.org/schema'})
out.append(E.Comment('Generated from cad_source/forklift.original.urdf. Unscaled CAD; see docs/forklift_model.md.'))
E.SubElement(out,'link',name='base_link')
def joint(name,typ,parent,child,xyz,rpy='0 0 0',axis=None):
 j=E.SubElement(out,'joint',name=name,type=typ);E.SubElement(j,'parent',link=parent);E.SubElement(j,'child',link=child);E.SubElement(j,'origin',xyz=xyz,rpy=rpy)
 if axis:E.SubElement(j,'axis',xyz=axis)
 return j
def box(link,name,xyz,size):
 c=E.SubElement(link,'collision',name=name);E.SubElement(c,'origin',xyz=xyz);E.SubElement(E.SubElement(c,'geometry'),'box',size=size)
links={l.get('name'):l for l in root.findall('link')}
# CAD root kept intact under a massless ROS root at floor level.
for name,l in links.items():
 l=E.fromstring(E.tostring(l));new='cad_base' if name=='base_link' else ('lift_carriage' if name=='can_Link' else name)
 wheel=name.startswith('banh') and name[4:5].isdigit()
 mapping={'banh1_Link':'rear_right_wheel','banh2_Link':'front_right_wheel','banh3_Link':'front_left_wheel','banh4_Link':'rear_left_wheel'}
 if wheel:new=mapping[name]
 l.set('name',new)
 for c in l.findall('collision'):l.remove(c)
 mesh=l.find('visual/geometry/mesh');mesh.set('filename','package://forklift_description/meshes/'+MESH_NAMES[name]+'.stl')
 if name=='base_link':
  joint('cad_base_fixed','fixed','base_link',new,f'0 0 {offset}')
  box(l,'chassis_collision','0 0 -0.01','0.3102 0.269439 0.05')
  box(l,'upper_deck_collision','0 0 0.155','0.3102 0.269439 0.015')
  box(l,'mast_collision','0.20649 0 0.09','0.02 0.09 0.175')
 elif wheel:
  # Normalize all wheel joint axes to +Y. Move origin along the same axle to
  # geometric mid-width; inverse visual/COM offsets preserve CAD assembly.
  s=1 if name in ('banh3_Link','banh4_Link') else -1;shift=-s*.026575
  for tag in ('visual','inertial'):
   o=l.find(tag+'/origin');v=list(map(float,o.get('xyz').split()));v[1]-=shift;o.set('xyz',' '.join(map(str,v)))
  old=next(j for j in root.findall('joint') if j.find('child').get('link')==name)
  xyz=list(map(float,old.find('origin').get('xyz').split()));xyz[1]+=shift
  j=joint(new+'_joint','continuous','cad_base',new,' '.join(map(str,xyz)),axis='0 1 0');E.SubElement(j,'dynamics',damping='0.001',friction='0')
  c=E.SubElement(l,'collision');E.SubElement(E.SubElement(c,'geometry'),'sphere',radius='0.05')
  g=E.SubElement(out,'gazebo',reference=new);fr=E.SubElement(E.SubElement(E.SubElement(E.SubElement(g,'collision'),'surface'),'friction'),'ode');E.SubElement(fr,'mu').text='1';E.SubElement(fr,'mu2').text='0';E.SubElement(fr,'fdir1',{'ignition:expressed_in':'base_link'}).text='1 -1 0' if name in ('banh1_Link','banh3_Link') else '1 1 0'
 elif name=='can_Link':
  # Bake CAD rotation into mesh origin and rotate its inertia to a vertical
  # prismatic-joint frame. CAD pose at q=0 is unchanged.
  t=T[name];j=joint('lift_joint','prismatic','base_link',new,' '.join(map(str,t[:3,3]+[0,0,offset])),axis='0 0 1');E.SubElement(j,'limit',lower="${-g['lift_sim_limit_margin']}",upper="${g['lift_travel']+g['lift_sim_limit_margin']}",velocity="${g['lift_sim_velocity_limit']}",effort='100');E.SubElement(j,'dynamics',damping='0',friction='0')
  l.find('visual/origin').set('rpy','1.5708 -1.5708 0')
  inert=l.find('inertial');o=inert.find('origin');v=t[:3,:3]@np.array(list(map(float,o.get('xyz').split())));o.set('xyz',' '.join(map(str,v)))
  q=inert.find('inertia');I=np.array([[float(q.get('ixx')),float(q.get('ixy')),float(q.get('ixz'))],[float(q.get('ixy')),float(q.get('iyy')),float(q.get('iyz'))],[float(q.get('ixz')),float(q.get('iyz')),float(q.get('izz'))]]);I=t[:3,:3]@I@t[:3,:3].T
  for key,a,b in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]:q.set(key,str(I[a,b]))
  box(l,'carriage_collision','-0.025 0 0.005','0.025 0.10 0.05')
  # CAD tines: x=.2313..4313, y bands [-.080918,-.065918]
  # and [.064082,.079082], z=.02373..02573 in original CAD base.
  for side,y in [('right',-.07341809),('left',.07158191)]:box(l,'fork_'+side+'_collision',f'0.098 {y+.00091809} -0.0275','0.20 0.015 0.002')
 else:
  old=next(j for j in root.findall('joint') if j.find('child').get('link')==name);p=old.find('parent').get('link');p='cad_base' if p=='base_link' else p
  # Unactuated transmission meshes remain fixed until screw lead is measured.
  joint(old.get('name'),'fixed',p,new,old.find('origin').get('xyz'),old.find('origin').get('rpy'))
 out.append(l)
# Normalize active names; original CAD identifiers remain only in source data.
LINK_NAMES={'cad_base':'chassis','lidar_Link':'lidar_housing','step_Link':'lift_transmission','banhrang1_Link':'lift_drive_gear','banhrang2_Link':'lift_screw'}
JOINT_NAMES={'cad_base_fixed':'chassis_fixed','lidar_joint':'lidar_housing_fixed','step_joint':'lift_transmission_fixed','banhrang1_joint':'lift_drive_gear_fixed','banhrang2_joint':'lift_screw_fixed'}
for element in out.iter():
 for key in ('name','link','reference'):
  value=element.get(key)
  if value in LINK_NAMES:element.set(key,LINK_NAMES[value])
  elif value in JOINT_NAMES:element.set(key,JOINT_NAMES[value])
for name,mesh_name in MESH_NAMES.items():
 if name!='base_link':shutil.copy2(S/'meshes'/f'{name}.STL',P/'meshes'/f'{mesh_name}.stl')
E.indent(out);(P/'urdf/forklift_cad.urdf.xacro').write_text(E.tostring(out,encoding='unicode')+'\n')
(R/'results/model_layout/cad_import.json').write_text(json.dumps({'original_triangles':len(base),'removed_duplicate_triangles':int(remove.sum()),'duplicates_by_child':counts,'mesh_scale':1,'cad_to_ground_z':offset,'note':'Original STL and URDF retained. CAD mass values retained, not experimentally validated.'},indent=2)+'\n')
print(counts, 'removed',remove.sum())
