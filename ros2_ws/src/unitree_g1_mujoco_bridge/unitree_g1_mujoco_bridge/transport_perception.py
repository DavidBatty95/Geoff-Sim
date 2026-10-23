"""Known-colour box/pad detections from registered RGB-D and measured robot state."""
import json
import math
import struct
import zlib
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo,JointState
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformException

from .onboard_lidar_odometry import lidar_extrinsics
from .arm_kinematics import rpy_matrix


def components(mask):
    remaining = set(map(tuple,np.argwhere(mask)))
    while remaining:
        first = remaining.pop()
        queue,group = [first],[first]
        while queue:
            y,x = queue.pop()
            for dy,dx in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
                point = (y+dy,x+dx)
                if point in remaining:
                    remaining.remove(point)
                    queue.append(point)
                    group.append(point)
        if len(group) >= 6:
            yield np.array(group)


def support_approach(points,box,max_edge_gap=.20,min_edge_separation=.08):
    """Face-normal approach from the visible support rectangle, not scene axes."""
    if len(points)<20 or not np.all(np.isfinite(points)) or not np.all(np.isfinite(box[:2])):
        return None
    lower,upper = np.min(points[:,:2],axis=0),np.max(points[:,:2],axis=0)
    if min(upper-lower)<.25 or np.any(np.asarray(box[:2])<lower-.04) or np.any(np.asarray(box[:2])>upper+.04):
        return None
    best = None
    for yaw in np.linspace(0.,math.pi/2,91)[:-1]:
        axes = np.array(((math.cos(yaw),-math.sin(yaw)),(math.sin(yaw),math.cos(yaw))))
        projected = points[:,:2]@axes
        lo,hi = np.min(projected,axis=0),np.max(projected,axis=0)
        area = float(np.prod(hi-lo))
        if best is None or area<best[0]:
            best = area,axes,lo,hi
    _,axes,lo,hi = best
    extent = hi-lo
    if min(extent)<.25 or max(extent)>2. or max(extent)<.45:
        return None
    local = np.asarray(box[:2])@axes
    if np.any(local<lo-.04) or np.any(local>hi+.04):
        return None
    gaps = [local[0]-lo[0],hi[0]-local[0],local[1]-lo[1],hi[1]-local[1]]
    edge = int(np.argmin(gaps))
    if gaps[edge]>max_edge_gap or sorted(gaps)[1]-gaps[edge]<min_edge_separation:
        return None  # No observed nearby edge to justify a tabletop approach.
    inward = axes[:,edge//2]*(1. if edge%2==0 else -1.)
    return math.atan2(inward[1],inward[0])


def support_surfaces(depth,K,origin,R,table_height=.95):
    """Bounded, colour-independent support proposals; never object discoveries."""
    v,u = np.indices(depth.shape)
    z = depth
    points = np.stack(((u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z),axis=-1)@R.T+origin
    height_band = (z>.3)&(z<6.)&(np.abs(points[:,:,2]-table_height)<.05)
    if min(z.shape)<3:
        return []
    # A horizontal height slice through a wall is not a tabletop. Require the
    # measured surface normal itself to be upright before connected components.
    normals = np.cross(np.gradient(points,axis=0),np.gradient(points,axis=1))
    lengths = np.linalg.norm(normals,axis=2)
    horizontal = height_band&(lengths>1e-8)&(np.abs(normals[:,:,2])>=.9*lengths)
    # A distant tabletop can occupy only 3--4 image rows. Decimating before
    # fitting destroyed its depth extent and made support search effectively
    # near-sighted. Recover height-consistent edge pixels beside plane normals,
    # then bound the point set after component extraction instead.
    padded = np.pad(horizontal,1)
    adjacent = np.logical_or.reduce([padded[y:y+z.shape[0],x:x+z.shape[1]]
                                    for y in range(3) for x in range(3)])
    mask = height_band&adjacent
    result = []
    for pixels in components(mask):
        surface = points[pixels[:,0],pixels[:,1]]
        if len(surface)>600:
            surface = surface[np.linspace(0,len(surface)-1,600,dtype=int)]
        lo,hi = np.min(surface,axis=0),np.max(surface,axis=0)
        extent = hi-lo
        xy = surface[:,:2]-np.mean(surface[:,:2],axis=0)
        _,axes = np.linalg.eigh(xy.T@xy)
        dimensions = np.ptp(xy@axes,axis=0)
        if .3<max(dimensions)<2.5 and min(dimensions)>.22:
            result.append(dict(position=((lo+hi)/2).tolist(),extent=extent.tolist(),pixels=len(surface)))
    return sorted(result,key=lambda item:-item['pixels'])[:16]


def colour_detections(rgb,depth,K,origin,R,table_height=.95,diagnostics=None,object_profile='legacy_cube'):
    """Return geometric candidates in a level robot frame; no layout lookup."""
    if (rgb.ndim!=3 or rgb.shape[2]!=3 or depth.shape!=rgb.shape[:2] or K.shape!=(3,3) or
            not np.all(np.isfinite(K)) or K[0,0]<=0. or K[1,1]<=0.):
        raise ValueError('RGB-D registration or camera intrinsics invalid')
    result = []
    for channel,colour in enumerate(('red','green','blue')):
        dominant = rgb[:,:,channel].astype(float)
        others = np.max(np.delete(rgb,channel,axis=2),axis=2)
        mask = (dominant > 65) & (dominant > 1.65*others) & (depth > .15) & (depth < 8.)
        for pixels in components(mask):
            v,u = pixels[:,0],pixels[:,1]
            z = depth[v,u]
            points = np.column_stack(((u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z))@R.T+origin
            low,high = np.quantile(points,.02,axis=0),np.quantile(points,.98,axis=0)
            # On a filled destination, separate the raised box from the outline.
            subsets = [points]
            if high[2]-low[2] > .05 and max(high[:2]-low[:2]) > .18:
                subsets = [points[points[:,2] < low[2]+.02],points[points[:,2] > low[2]+.025]]
            for subset in subsets:
                if len(subset) < 6:
                    continue
                lo,hi = np.min(subset,axis=0),np.max(subset,axis=0)
                extent = hi-lo
                centre = .5*(lo+hi)
                if diagnostics is not None:
                    diagnostics.append(dict(colour=colour,low=lo.tolist(),high=hi.tolist(),pixels=len(subset)))
                # A top-only sliver or a short occluded side fragment does not
                # establish the centre of the 90 mm upright grasp cylinder.
                small_object = (.012 < max(extent[:2]) < .075 and .06<extent[2]<.12) if object_profile=='dex3_cylinder' else (.045 < max(extent[:2]) < .19 and extent[2]<.18)
                if small_object:
                    kind = 'box'
                    centre[2] = hi[2]-(.045 if object_profile=='dex3_cylinder' else .055)
                    if object_profile=='dex3_cylinder':
                        wall = subset[(subset[:,2]<hi[2]-.008)&(subset[:,2]>lo[2]+.008),:2]
                        if len(wall)>=6:
                            design = np.column_stack((2*wall,np.ones(len(wall))))
                            fit,_,rank,_ = np.linalg.lstsq(design,np.sum(wall*wall,axis=1),rcond=None)
                            radius = math.sqrt(max(0.,float(fit[2]+fit[0]**2+fit[1]**2)))
                            if rank==3 and .012<=radius<=.03:
                                centre[:2] = fit[:2]
                    if centre[2] < table_height+.02:
                        continue
                elif .30 < max(extent[:2]) < .58 and min(extent[:2]) > .30 and extent[2] < .045:
                    kind = 'pad'
                else:
                    continue
                detection = dict(id=('box_' if kind=='box' else 'target_')+colour,
                                 kind=kind,colour=colour,position=centre.tolist(),extent=extent.tolist(),pixels=len(subset))
                if kind == 'pad':
                    # Minimum-area rectangle supplies the observed square's
                    # edge directions, allowing a face-normal table approach.
                    best = None
                    for yaw in np.linspace(0.,math.pi/2,91)[:-1]:
                        axes = np.array(((math.cos(yaw),-math.sin(yaw)),(math.sin(yaw),math.cos(yaw))))
                        projected = subset[:,:2]@axes
                        bounds = np.min(projected,axis=0),np.max(projected,axis=0)
                        area = float(np.prod(bounds[1]-bounds[0]))
                        if best is None or area < best[0]:
                            best = (area,float(yaw),(.5*(bounds[0]+bounds[1]))@axes.T)
                    detection['yaw'] = best[1]
                    detection['position'][:2] = best[2].tolist()
                result.append(detection)
    supported = [item for item in result if item['kind'] in ('box','pad')]
    if supported:
        v,u = np.indices(depth.shape)
        points = np.stack(((u-K[0,2])*depth/K[0,0],(v-K[1,2])*depth/K[1,1],depth),axis=-1)@R.T+origin
        # Support orientation needs broad plane coverage, not every pixel.
        # Decimation bounds Python connected-component work while cube/pad
        # detection above retains full registered resolution.
        points = points[::2,::2]
        support_depth = depth[::2,::2]
        support = ((support_depth>.15)&(support_depth<8.)&(points[:,:,2]>table_height-.03)&
                   (points[:,:,2]<table_height+.07))
        approaches = {id(item):[] for item in supported}
        for pixels in components(support):
            surface = points[pixels[:,0],pixels[:,1]]
            clipped = bool(np.any(pixels[:,0]==0) or np.any(pixels[:,0]==support.shape[0]-1) or
                           np.any(pixels[:,1]==0) or np.any(pixels[:,1]==support.shape[1]-1))
            for item in supported:
                if item['kind']=='pad' and clipped:
                    continue  # An image border is not an observed tabletop edge.
                approach = support_approach(surface,item['position'],max_edge_gap=.30 if item['kind']=='pad' else .20)
                if approach is not None:
                    approaches[id(item)].append(approach)
        for item in supported:
            choices = approaches[id(item)]
            if choices and all(abs(math.atan2(math.sin(yaw-choices[0]),math.cos(yaw-choices[0])))<=.15 for yaw in choices):
                item['approach_yaw'] = choices[0]
                item['approach_source'] = 'rgbd_support_edge'
    return result


class ObservationConsensus:
    """Confirm stationary geometry across distinct image acquisitions.

    Ambiguity or a pose jump revokes the old estimate immediately. Missing views
    retain only the original acquisition stamp for bounded occlusion memory.
    """
    def __init__(self,max_shift=.06,max_interval=1.5):
        self.max_shift,self.max_interval = max_shift,max_interval
        self.pending,self.confirmed = {},{}
        self.latest_stamp = -math.inf

    def update(self,detections,stamp):
        # Replayed or delayed frames must not revoke newer confirmations or
        # prune their memory using an older acquisition time.
        if not math.isfinite(stamp) or stamp<=self.latest_stamp:
            return dict(self.confirmed)
        self.latest_stamp = stamp
        grouped = {}
        for item in detections:
            grouped.setdefault(item['id'],[]).append(item)
        for identifier,items in grouped.items():
            if len(items)!=1:
                self.pending.pop(identifier,None)
                self.confirmed.pop(identifier,None)
                continue
            item = dict(items[0])
            point = np.asarray(item['position'],dtype=float)
            if point.shape!=(3,) or not np.all(np.isfinite(point)) or not math.isfinite(stamp):
                self.pending.pop(identifier,None)
                self.confirmed.pop(identifier,None)
                continue
            previous = self.pending.get(identifier)
            if previous is not None and stamp<=previous['observed_stamp']:
                continue
            consistent = (previous is not None and stamp-previous['observed_stamp']<=self.max_interval and
                          np.linalg.norm(point-np.asarray(previous['position']))<=self.max_shift)
            count = min(10,previous.get('observation_count',1)+1) if consistent else 1
            item.update(observed_stamp=stamp,observation_count=count,confirmed=count>=2)
            approach_count = 0
            if 'approach_yaw' in item and math.isfinite(item['approach_yaw']):
                approach_count = 1
                if consistent and 'approach_yaw' in previous:
                    error = item['approach_yaw']-previous['approach_yaw']
                    if abs(math.atan2(math.sin(error),math.cos(error)))<=.15:
                        approach_count = min(10,previous.get('approach_observation_count',1)+1)
            item['approach_observation_count'] = approach_count
            self.pending[identifier] = item
            if count>=2:
                self.confirmed[identifier] = dict(item)
                if approach_count<2:
                    self.confirmed[identifier].pop('approach_yaw',None)
                    self.confirmed[identifier].pop('approach_source',None)
            else:
                self.confirmed.pop(identifier,None)
        self.confirmed = {key:item for key,item in self.confirmed.items()
                          if 0.<=stamp-item['observed_stamp']<(60. if item['kind']=='pad' else 30.)}
        self.pending = {key:item for key,item in self.pending.items() if 0.<=stamp-item['observed_stamp']<60.}
        return dict(self.confirmed)


class TransportPerception(Node):
    def __init__(self,camera_pitch_deg=0.,object_profile='dex3_cylinder'):
        self.object_profile = object_profile
        super().__init__('transport_perception')
        qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        image_qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
        self.info = None
        self.colour = {}
        self.depth = {}
        self.imu,self.joints = deque(maxlen=200),deque(maxlen=200)
        self.objects = {}
        self.surfaces = []
        self.image_pose = None
        self.image_hfov = math.pi/3
        self.consensus = ObservationConsensus()
        self.camera_pitch_deg = float(camera_pitch_deg)
        self.snapshot = None
        self.frame_history = deque(maxlen=180)
        self.attempt_history = deque(maxlen=600)
        self.last_processed_stamp = -math.inf
        self.process_status = 'waiting_for_images'
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.pub = self.create_publisher(String,'/transport/observations',10)
        self.create_subscription(CameraInfo,'/camera/camera/color/camera_info',self._info,10)
        self.create_subscription(Image,'/camera/camera/color/image_raw',self._colour,image_qos)
        self.create_subscription(Image,'/camera/camera/aligned_depth_to_color/image_raw',self._depth,image_qos)
        self.create_subscription(Vector3Stamped,'/onboard/measured_rpy',lambda msg:self.imu.append((self.stamp(msg.header),(msg.vector.x,msg.vector.y,msg.vector.z))),qos)
        self.create_subscription(JointState,'/joint_states',lambda msg:self.joints.append((self.stamp(msg.header),dict(zip(msg.name,msg.position)))),qos)
        self.create_timer(.2,self._publish)

    def seconds(self):
        return self.get_clock().now().nanoseconds*1e-9

    @staticmethod
    def stamp(header):
        return header.stamp.sec+header.stamp.nanosec*1e-9

    def _info(self,msg):
        self.info = np.array(msg.k).reshape(3,3)

    def _colour(self,msg):
        if msg.encoding == 'rgb8':
            self.colour[self.stamp(msg.header)] = msg
            self.colour = dict(list(self.colour.items())[-5:])
            self._process(self.stamp(msg.header))

    def _depth(self,msg):
        self.depth[self.stamp(msg.header)] = msg
        self.depth = dict(list(self.depth.items())[-5:])
        self._process(self.stamp(msg.header))

    def _process(self,stamp):
        if stamp<=self.last_processed_stamp:
            self.colour.pop(stamp,None)
            self.depth.pop(stamp,None)
            return
        self._process_frame(stamp)
        self.attempt_history.append(dict(stamp=stamp,processed_at=self.seconds(),status=self.process_status,
            latest_imu_stamp=self.imu[-1][0] if self.imu else None,
            latest_joint_stamp=self.joints[-1][0] if self.joints else None))

    def _process_frame(self,stamp):
        msg = self.depth.get(stamp)
        if msg is None:
            self.process_status = 'waiting_for_depth'
            return
        colour = self.colour.get(stamp)
        if colour is None or self.info is None or not self.imu or not self.joints or msg.encoding != '16UC1':
            self.process_status = 'waiting_for_registered_pair_or_robot_state'
            return
        imu = min(self.imu,key=lambda p:abs(p[0]-stamp))
        joints = min(self.joints,key=lambda p:abs(p[0]-stamp))
        if abs(imu[0]-stamp)>.2 or abs(joints[0]-stamp)>.2:
            self.process_status = f'state_gap: imu={abs(imu[0]-stamp):.3f}, joints={abs(joints[0]-stamp):.3f}'
            return
        try:
            transform = self.tf.lookup_transform('odom','base_footprint',rclpy.time.Time.from_msg(msg.header.stamp)).transform
        except TransformException as exc:
            self.process_status = str(exc)
            return
        self.process_status = 'processed'
        rgb = np.frombuffer(colour.data,dtype=np.uint8).reshape(colour.height,colour.step)[:,:colour.width*3].reshape(colour.height,colour.width,3)
        depth = np.frombuffer(msg.data,dtype='<u2').reshape(msg.height,msg.step//2)[:,:msg.width].astype(float)*.001
        origin,lidar_R = lidar_extrinsics(imu[1],joints[1])
        torso_R = lidar_R@np.diag((1.,-1.,-1.))
        origin = origin+torso_R@np.array((0.,.035,0.))
        optical_R = np.array(((0.,0.,1.),(-1.,0.,0.),(0.,-1.,0.)))
        q = transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        world_R = rpy_matrix((0.,0.,yaw))
        offset = np.array((transform.translation.x,transform.translation.y,0.))
        diagnostics = []
        try:
            detections = colour_detections(rgb,depth,self.info,origin,
                torso_R@rpy_matrix((0.,math.radians(self.camera_pitch_deg),0.))@optical_R,diagnostics=diagnostics,object_profile=self.object_profile)
        except ValueError as exc:
            self.process_status = str(exc)
            return
        self.surfaces = support_surfaces(depth,self.info,origin,
            torso_R@rpy_matrix((0.,math.radians(self.camera_pitch_deg),0.))@optical_R)
        for surface in self.surfaces:
            surface['position'] = (world_R@surface['position']+offset).tolist()
            surface['observed_stamp'] = stamp
        camera_origin = world_R@origin+offset
        camera_rotation = world_R@torso_R@rpy_matrix((0.,math.radians(self.camera_pitch_deg),0.))@optical_R
        self.image_pose = [float(camera_origin[0]),float(camera_origin[1]),
                           math.atan2(camera_rotation[1,2],camera_rotation[0,2])]
        self.image_hfov = 2.*math.atan(colour.width/(2.*self.info[0,0]))
        self.snapshot = dict(rgb=rgb.copy(),depth=depth.copy(),stamp=stamp,origin=origin.tolist(),
                             rpy=list(imu[1]),torso_rotation=torso_R.tolist(),regions=diagnostics,detections=detections)
        for item in detections:
            item['position'] = (world_R@item['position']+offset).tolist()
            if 'yaw' in item:
                item['yaw'] += yaw
            if 'approach_yaw' in item:
                item['approach_yaw'] += yaw
            item['observed_stamp'] = stamp
        self.objects = self.consensus.update(detections,stamp)
        self.frame_history.append(dict(stamp=stamp,detected=[item['id'] for item in detections],
            confirmed=list(self.objects),regions=diagnostics,detections_odom=detections,
            pose_odom=[*offset[:2],yaw],imu_stamp=imu[0],joint_stamp=joints[0],
            processed_at=self.seconds(),
            acquisition_interval_s=stamp-self.last_processed_stamp if math.isfinite(self.last_processed_stamp) else None))
        self.last_processed_stamp = stamp
        self.colour.pop(stamp,None)
        self.depth.pop(stamp,None)

    def save_snapshot(self,directory):
        directory.mkdir(parents=True,exist_ok=True)
        (directory/'perception_history.json').write_text(json.dumps(list(self.frame_history),indent=2)+'\n')
        (directory/'perception_attempts.json').write_text(json.dumps(list(self.attempt_history),indent=2)+'\n')
        if self.snapshot is None:
            return
        snapshot = dict(self.snapshot)
        snapshot['camera_pitch_deg'] = self.camera_pitch_deg
        snapshot.update(process_status=self.process_status,colour_stamps=list(self.colour),depth_stamps=list(self.depth),
                        latest_imu_stamp=self.imu[-1][0] if self.imu else None,
                        latest_joint_stamp=self.joints[-1][0] if self.joints else None,
                        snapshot_written_at=self.seconds())
        rgb,depth = snapshot.pop('rgb'),snapshot.pop('depth')
        np.savez_compressed(directory/'rgbd.npz',rgb=rgb,depth=depth,K=self.info)
        height,width,_ = rgb.shape
        def chunk(kind,data):
            return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
        raw = b''.join(b'\0'+row.tobytes() for row in rgb)
        png = b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',width,height,8,2,0,0,0))
        (directory/'rgb.png').write_bytes(png+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))
        (directory/'perception_snapshot.json').write_text(json.dumps(snapshot,indent=2)+'\n')

    def _publish(self):
        now = self.seconds()
        for stamp in list(self.depth):
            if 0. <= now-stamp <= 1.5:
                self._process(stamp)
        self.objects = {key:value for key,value in self.objects.items() if now-value['observed_stamp'] < (60. if value['kind']=='pad' else 30.)}
        self.pub.publish(String(data=json.dumps(dict(frame='odom',stamp=now,
            image_stamp=self.last_processed_stamp if math.isfinite(self.last_processed_stamp) else None,
            image_pose=self.image_pose,image_hfov=self.image_hfov,surfaces=self.surfaces,
            objects=list(self.objects.values()),source='registered_rgbd_joint_imu'))))
