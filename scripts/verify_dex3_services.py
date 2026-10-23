#!/usr/bin/env python3
"""Exercise ROS grasp/release services through the native Dex3 DDS contact rig."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from g1_msgs.srv import GraspObject

from unitree_g1_mujoco_bridge.sim_manipulation_io import _create_node_class


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--side',choices=('left','right'),default='right')
    parser.add_argument('--expect-no-grasp',action='store_true',help='Negative control with object beyond finger reach')
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    os.environ['CYCLONEDDS_URI'] = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="localhost"/></Peers></Discovery></Domain></CycloneDDS>'
    rclpy.init(args=['--ros-args','-p','object_state_socket:=/tmp/dex3_verify_objects.sock',
        '-p','command_response_socket:=/tmp/dex3_verify_response.sock'])
    cls,_ = _create_node_class()
    server = cls()
    client = Node('dex3_service_verifier')
    executor = SingleThreadedExecutor()
    executor.add_node(server);executor.add_node(client)
    grasp = client.create_client(GraspObject,'/g1_sim_manipulation_io/grasp_object')
    release = client.create_client(Trigger,f'/g1_sim_manipulation_io/release_{args.side}')
    native_result=args.output/'native_result.json'
    result = dict(scope='ROS service + native DDS contact bench; not a mobile mission',side=args.side,
                  expected_grasp_success=not args.expect_no_grasp,events=[])
    process = None
    try:
        with (args.output/'native.log').open('w') as log:
            environment = dict(os.environ,LD_LIBRARY_PATH='/workspace/vendor/unitree_sdk2_install/lib:'+os.environ.get('LD_LIBRARY_PATH',''))
            process = subprocess.Popen(['/workspace/ros2_ws/src/unitree_mujoco/simulate/build/dex3_contact_bench',str(args.fixture),str(native_result)],stdout=log,stderr=subprocess.STDOUT,env=environment)
            deadline=time.monotonic()+30.
            grasp_future=release_future=None
            while time.monotonic()<deadline and process.poll() is None:
                executor.spin_once(timeout_sec=.01)
                tick=server.dex3.sequence.get(args.side,0)
                if tick>=1500 and grasp_future is None and grasp.service_is_ready():
                    grasp_future=grasp.call_async(GraspObject.Request(arm=args.side,object_id='bench_cylinder'))
                if tick>=11500 and release_future is None and release.service_is_ready():
                    release_future=release.call_async(Trigger.Request())
            if process.poll() is None:
                raise RuntimeError('native hand bench exceeded deadline')
            for name,future in (('grasp',grasp_future),('release',release_future)):
                response=future.result() if future is not None and future.done() else None
                result['events'].append(dict(operation=name,success=bool(response and response.success),
                                              message=response.message if response else 'missing response'))
            result['native']=json.loads(native_result.read_text()) if native_result.exists() else {}
            result['success']=bool(result['native'].get('success') and all(row['success'] for row in result['events']))
            if args.expect_no_grasp:
                result['success']=bool(result['native'] and not result['native'].get('success') and
                    not result['events'][0]['success'] and result['events'][1]['success'])
            result['returncode']=process.returncode
            result['success'] = bool(result['success'] and process.returncode==(1 if args.expect_no_grasp else 0))
    finally:
        if process is not None and process.poll() is None:
            process.terminate();process.wait(timeout=10)
        executor.shutdown();server.destroy_node();client.destroy_node();rclpy.shutdown()
        (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return int(not result['success'])


if __name__=='__main__':
    raise SystemExit(main())
