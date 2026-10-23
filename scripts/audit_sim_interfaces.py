#!/usr/bin/env python3
"""Passively check live G1 ROS message contracts and measured sensor delivery."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo,Image,Imu,PointCloud2
from unitree_hg.msg import LowCmd,LowState,HandState
from unitree_go.msg import MotorCmds,MotorStates
from tf2_ros import Buffer,TransformListener

from unitree_g1_mujoco_bridge.unitree_crc import lowcmd_crc

CONTRACT = {
    '/lowstate':LowState,'/lowcmd':LowCmd,
    '/camera/camera/color/image_raw':Image,
    '/camera/camera/color/camera_info':CameraInfo,
    '/camera/camera/aligned_depth_to_color/image_raw':Image,
    '/camera/camera/imu':Imu,'/livox/lidar':PointCloud2,'/livox/imu':Imu,
}


class Audit(Node):
    def __init__(self,hands,hand_model='dex3'):
        super().__init__('sim_hardware_interface_audit')
        self.errors = set()
        self.observed = {}
        self.mode = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.contract = dict(CONTRACT)
        if hands:
            for side in ('left','right'):
                self.contract[f'/lf/dex3/{side}/state' if hand_model=='dex3' else f'/brainco/{side}/state'] = HandState if hand_model=='dex3' else MotorStates
        for topic,kind in self.contract.items():
            self.create_subscription(kind,topic,lambda message,topic=topic:self.sample(topic,message),qos_profile_sensor_data)

    def sample(self,topic,msg):
        now = time.monotonic()
        entry = self.observed.setdefault(topic,dict(count=0,first=now,last=now,ages=[],duplicate_stamps=0))
        entry['count'] += 1
        entry['last'] = now
        if topic=='/lowstate':
            self.mode = int(msg.mode_machine)
            if len(msg.motor_state)!=35:
                self.errors.add('HG motor-state array is not 35')
            entry['mode_machine'] = self.mode
            entry['max_body_tilt_rad'] = float(max(entry.get('max_body_tilt_rad',0.),abs(msg.imu_state.rpy[0]),abs(msg.imu_state.rpy[1])))
        if topic=='/lowcmd' and entry['count']%10==1:
            if lowcmd_crc(msg)!=msg.crc:
                self.errors.add('invalid LowCmd CRC')
            if self.mode is not None and msg.mode_machine!=self.mode:
                self.errors.add('LowCmd machine mode differs from LowState')
        if hasattr(msg,'header'):
            stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            if stamp==entry.get('previous_stamp'):
                entry['duplicate_stamps'] += 1
            entry['previous_stamp'] = stamp
            entry['frame'] = msg.header.frame_id
            age = (self.get_clock().now().nanoseconds-stamp)*1e-6
            entry['ages'].append(age)
            if stamp<=0 or age<0:
                self.errors.add(topic+': invalid acquisition timestamp')
        if isinstance(msg,Imu):
            if msg.orientation_covariance[0]!=-1.:
                self.errors.add(topic+': raw IMU advertises an attitude solution')
            values = [msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z,
                      msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
            if not all(np.isfinite(values)):
                self.errors.add(topic+': nonfinite measured IMU')
            entry['last_gyro_accel'] = values
        if isinstance(msg,CameraInfo):
            if list(msg.r)!=[1.,0.,0.,0.,1.,0.,0.,0.,1.] or msg.k[0]<=0 or msg.k[4]<=0:
                self.errors.add('invalid camera calibration')
            entry['resolution'] = [msg.width,msg.height]
        if isinstance(msg,Image):
            expected = 'rgb8' if '/color/image_raw' in topic else '16UC1'
            if msg.encoding!=expected or len(msg.data)!=msg.height*msg.step:
                self.errors.add(topic+': encoding/stride mismatch')
        if isinstance(msg,PointCloud2):
            entry['fields'] = [[field.name,field.offset,field.datatype] for field in msg.fields]
            entry['point_step'] = msg.point_step
        if isinstance(msg,MotorStates):
            if len(msg.states)!=6 or any(not 0.<=state.q<=1. for state in msg.states):
                self.errors.add(topic+': invalid normalized six-channel hand state')
        if isinstance(msg,HandState):
            if len(msg.motor_state)!=7 or any(not np.isfinite(state.q) for state in msg.motor_state):
                self.errors.add(topic+': invalid seven-channel Dex3 radian state')
            entry['controller_error_bits'] = list(msg.error)
            entry['simulated_contact_proxy'] = msg.reserve[1]==0x44583343

    def report(self):
        graph = dict(self.get_topic_names_and_types())
        for topic,kind in self.contract.items():
            expected = kind.__module__.split('.')[0]+'/msg/'+kind.__name__
            if expected not in graph.get(topic,[]):
                self.errors.add(topic+': expected type absent: '+expected)
            if self.observed.get(topic,{}).get('count',0)<2:
                self.errors.add(topic+': insufficient samples')
        for alias in ('/rt/lowcmd','/rt/lowstate'):
            if alias in graph:
                self.errors.add('unexpected legacy ROS rt/ alias: '+alias)
        result = {}
        for topic,data in self.observed.items():
            age = data.pop('ages')
            first,last = data.pop('first'),data.pop('last')
            data.pop('previous_stamp',None)
            data['observed_rate_hz'] = (data['count']-1)/(last-first) if last>first else 0.
            data['age_ms'] = dict(median=float(np.median(age)),p95=float(np.quantile(age,.95)),maximum=max(age)) if age else None
            if data.get('frame'):
                data['measured_tf_available'] = self.tf.can_transform('base_footprint',data['frame'],rclpy.time.Time())
                if not data['measured_tf_available']:
                    self.errors.add(topic+': no measured sensor transform')
            result[topic] = data
        return dict(passed=not self.errors,scope='live ROS contract check, not physical calibration',
                    errors=sorted(self.errors),topics=result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=20.)
    parser.add_argument('--require-hands',action='store_true')
    parser.add_argument('--hand-model',choices=('dex3','brainco'),default='dex3')
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Audit(args.require_hands,args.hand_model)
    deadline = time.monotonic()+args.duration
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        result = node.report()
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
        return 0 if result['passed'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
