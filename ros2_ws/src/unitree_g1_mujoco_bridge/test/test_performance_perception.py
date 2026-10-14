import struct
import math
from types import SimpleNamespace as S

import numpy as np
from sensor_msgs.msg import PointCloud2,PointField

from unitree_g1_mujoco_bridge.dynamic_obstacle_tracker import DynamicObstacleTracker
from unitree_g1_mujoco_bridge.transport_perception import support_surfaces


def test_tracker_decodes_big_endian_organized_rows_and_noise_tags():
    fields = [PointField(name=name,offset=4*i,datatype=PointField.FLOAT32,count=1)
              for i,name in enumerate(('x','y','z'))]
    fields.append(PointField(name='tag',offset=12,datatype=PointField.UINT8,count=1))
    msg = PointCloud2(width=1,height=2,point_step=16,row_step=20,is_bigendian=True,fields=fields,
        data=struct.pack('>fffB7x',1.,2.,3.,0)+struct.pack('>fffB7x',4.,5.,6.,4))
    assert list(DynamicObstacleTracker._iter_cloud_points(S(use_livox_tag_filter=True),msg))==[
        (1.,2.,3.,False),(4.,5.,6.,True)]


def test_support_proposals_do_not_require_coloured_objects():
    depth = np.ones((64,64))*.95
    K = np.array(((80.,0.,32.),(0.,80.,32.),(0.,0.,1.)))
    surfaces = support_surfaces(depth,K,np.zeros(3),np.eye(3))
    assert len(surfaces)==1 and surfaces[0]['extent'][0]>.3
    assert not support_surfaces(depth,K,np.array((0.,0.,1.)),np.eye(3))


def test_distant_table_with_only_a_few_image_rows_still_generates_a_support_view():
    K = np.array(((100.,0.,95.5),(0.,100.,53.5),(0.,0.,1.)))
    v,u = np.indices((108,192))
    c,s = math.cos(math.radians(25)),math.sin(math.radians(25))
    R = np.array(((c,0.,s),(0.,1.,0.),(-s,0.,c)))@np.array(((0.,0.,1.),(-1.,0.,0.),(0.,-1.,0.)))
    origin = np.array((0.,0.,1.25))
    rays = np.stack(((u-K[0,2])/100.,(v-K[1,2])/100.,np.ones_like(u)),axis=-1)@R.T
    distance = (.95-origin[2])/rays[:,:,2]
    points = origin+distance[:,:,None]*rays
    valid = (distance>0.)&(points[:,:,0]>=2.5)&(points[:,:,0]<=3.5)&(np.abs(points[:,:,1])<=.55)
    depth = np.where(valid,distance,0.)
    assert 2<=np.count_nonzero(np.any(valid,axis=1))<=6
    surfaces = support_surfaces(depth,K,origin,R)
    assert len(surfaces)==1 and surfaces[0]['extent'][0]>.4
