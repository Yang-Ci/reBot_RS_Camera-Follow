"""Generate nominal V1 level camera models from the CAD mount transform."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

root=Path(__file__).resolve().parents[1] / 'models/wrist_camera'
twin=Path(__file__).resolve().parents[1] / 'assets/wrist-cameras/urdf'
def matrix(node):
    origin=node.find('origin'); out=np.eye(4)
    out[:3,:3]=Rotation.from_euler('xyz', [float(x) for x in origin.get('rpy','0 0 0').split()]).as_matrix()
    out[:3,3]=[float(x) for x in origin.get('xyz','0 0 0').split()]
    return out

def set_matrix(joint, transform):
    origin=joint.find('origin')
    origin.set('xyz',' '.join(f'{v:.10f}' for v in transform[:3,3]))
    origin.set('rpy',' '.join(f'{v:.10f}' for v in Rotation.from_matrix(transform[:3,:3]).as_euler('xyz')))

def replace_mount(urdf):
    visual=urdf.find("link[@name='camera_mount_link']/visual")
    visual.find('origin').set('xyz','0.0002 -0.04472 0.03452')
    visual.find('origin').set('rpy','0 0 0')
    mesh=visual.find('geometry/mesh')
    mesh.set('filename','package://rebot_mount/gemini2-v1-level/Gemini2_V1_Level.stl')
    mesh.set('scale','0.001 0.001 0.001')

def save(urdf,path):
    ET.indent(urdf,space='  ')
    ET.ElementTree(urdf).write(path,encoding='unicode',xml_declaration=True)

# CAD reports a -14.999820349° X rotation around (0,40,-11) mm.
# The V1 mesh is translated into mount coordinates by (0.2,-44.72,34.52) mm.
pivot=np.array([.0002,-.00472,.02352])
rot=Rotation.from_euler('x',-14.999820349448367,degrees=True).as_matrix()
level=np.eye(4);level[:3,:3]=rot;level[:3,3]=pivot-rot@pivot
v336=ET.parse(root/'gemini336.urdf').getroot()
v2=ET.parse(twin/'gemini2.urdf').getroot()
d435=ET.parse(twin/'d435i.urdf').getroot()
j336=v336.find("joint[@name='mount_to_camera']")
j2=v2.find("joint[@name='mount_to_camera']")
j435=d435.find("joint[@name='camera_joint']")
# Nominal V1 alignment from the existing 336 model. The camera-link origins
# differ by 0.96 mm along the camera x axis; the seating surfaces coincide.
v1_336=matrix(j336)
link_adjust=np.eye(4);link_adjust[0,3]=-.00096
v1_2=v1_336@link_adjust
old_2=matrix(j2)
old_to_v1=v1_2@np.linalg.inv(old_2)
set_matrix(j336,level@v1_336)
set_matrix(j2,level@v1_2)
set_matrix(j435,level@old_to_v1@matrix(j435))
for model in (v336,v2,d435): replace_mount(model)
# Keep only the body visual from the twin; secondary frame visuals duplicate
# the camera and are unnecessary for a dashboard view.
for model in (v2,d435):
    for link in model.findall('link'):
        if link.get('name') not in ('camera_link','camera_mount_link'):
            for visual in link.findall('visual'): link.remove(visual)
# The twin provides Gemini 2 STL assets; use the same web resource mapping.
for mesh in v2.findall('.//mesh'):
    filename=mesh.get('filename','')
    if filename.startswith('../meshes/'):
        mesh.set('filename','package://twin_camera/meshes/'+filename[len('../meshes/'):])
# Existing twin D435 body is a DAE mesh, unsupported by the STL-only loader.
# Its documented collision box gives a nominal representation of the body.
visual=d435.find("link[@name='camera_link']/visual")
visual.find('origin').set('rpy','0 0 0');visual.find('origin').set('xyz','0 -0.0175 0')
geometry=visual.find('geometry');geometry.remove(geometry.find('mesh'))
ET.SubElement(geometry,'box',{'size':'0.02505 0.09 0.025'})
for model,name in ((v336,'gemini336_level.urdf'),(v2,'gemini2_level.urdf'),(d435,'d435i_level.urdf')):
    save(model,root/name)
    print(name,'created')
