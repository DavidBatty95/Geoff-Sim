"""Bounded, read-only capture of resolved ROS parameters, not launch arguments."""
from collections import Counter
import math
import time

from rcl_interfaces.srv import GetParameters,ListParameters
from rclpy.parameter import parameter_value_to_python

TRANSPORT_REQUIRED_NODES = (
    '/g1_mujoco_bridge','/g1_policy_cmd_vel_controller','/g1_sim_sensors',
    '/onboard_lidar_odometry','/dynamic_obstacle_tracker',
    '/human_aware_motion_planner','/human_collision_avoidance',
    '/g1_arm_command_compositor','/g1_sim_manipulation_io',
    '/controller_server','/planner_server','/slam_toolbox',
)

COMMAND_PARAMETERS = (
    'max_lin_vel_x','max_lin_vel_y','max_ang_vel_z','max_planar_speed',
    'cmd_accel_x','cmd_accel_y','cmd_accel_yaw','cmd_vel_deadband',
    'cmd_vel_deadband_hysteresis','turning_arc_min_forward_speed','turning_arc_min_yaw_speed',
    'axis_exclusive_cmd_vel','axis_exclusive_yaw_threshold','allow_reverse_cmd_vel',
)
RESPONSE_PARAMETERS = (
    'response_profile',
    'response_forward_gain','response_lateral_gain','response_yaw_gain',
    'response_linear_time_constant','response_yaw_time_constant','response_lateral_yaw_gain',
    'response_walking_yaw_gain','response_negative_lateral_gain','response_positive_lateral_time_constant',
)


def compare_motion_parameters(snapshot):
    """Check resolved shared contracts, not similarly named source YAML entries."""
    planner,supervisor,policy = ('/human_aware_motion_planner','/human_collision_avoidance',
                                  '/g1_policy_cmd_vel_controller')
    differences = []
    checked = 0
    for keys,nodes in ((COMMAND_PARAMETERS,(planner,supervisor,policy)),
                       (RESPONSE_PARAMETERS,(planner,supervisor))):
        for key in keys:
            values = {}
            missing = []
            for name in nodes:
                record = snapshot.get('nodes',{}).get(name,{})
                parameter = record.get('parameters',{}).get(key)
                if record.get('status') != 'captured' or parameter is None:
                    missing.append(name)
                else:
                    values[name] = parameter['value']
            checked += 1
            if missing or any(value != next(iter(values.values())) for value in values.values()):
                differences.append(dict(parameter=key,values=values,missing_nodes=missing))
    return dict(consistent=not differences,checked_parameters=checked,differences=differences,
                scope='command bounds/filter across three nodes; prediction response across planner/supervisor')


def parameter_record(value):
    converted = parameter_value_to_python(value)
    if value.type == 5:  # Byte arrays may contain single-byte bytes objects.
        converted = [item[0] if isinstance(item,bytes) else int(item) for item in converted]
    def finite(item):
        if isinstance(item,float) and not math.isfinite(item):
            return dict(nonfinite=str(item))
        if isinstance(item,list):
            return [finite(part) for part in item]
        return item
    return dict(type=int(value.type),value=finite(converted))


def capture_parameters(node,pump,timeout=10.,required_nodes=TRANSPORT_REQUIRED_NODES,optional_timeout=2.):
    """Read all discovered nodes concurrently within one monotonic deadline.

    Call outside a ROS callback. ``pump(seconds)`` services the owning executor.
    Missing/duplicate nodes and incomplete responses are recorded, never silently
    substituted with launch defaults. This is a startup snapshot, not continuous
    parameter-change monitoring or an atomic distributed snapshot.
    """
    started = time.monotonic()
    started_ros = node.get_clock().now().nanoseconds*1e-9
    names = Counter(namespace.rstrip('/')+'/'+name
                    for name,namespace in node.get_node_names_and_namespaces())
    records = {name:dict(status='waiting_for_services') for name in sorted(set(names)|set(required_nodes))}
    jobs = {}
    clients = []
    try:
        for name,record in records.items():
            if names[name] == 0:
                record['status'] = 'missing_node'
            elif names[name] > 1:
                record['status'] = 'duplicate_node_name'
            else:
                listing = node.create_client(ListParameters,name+'/list_parameters')
                getter = node.create_client(GetParameters,name+'/get_parameters')
                clients.extend((listing,getter))
                jobs[name] = dict(listing=listing,getter=getter,future=None,names=None)
        while jobs and time.monotonic()-started < timeout:
            for name,job in list(jobs.items()):
                record = records[name]
                if name not in required_nodes and time.monotonic()-started >= optional_timeout:
                    record.update(status='optional_timeout',pending_stage=record['status'])
                    del jobs[name]
                    continue
                try:
                    if job['future'] is None:
                        if not (job['listing'].service_is_ready() and job['getter'].service_is_ready()):
                            continue
                        job['future'] = job['listing'].call_async(ListParameters.Request())
                        record['status'] = 'listing'
                    elif job['future'].done():
                        response = job['future'].result()
                        if job['names'] is None:
                            job['names'] = sorted(response.result.names)
                            job['future'] = job['getter'].call_async(GetParameters.Request(names=job['names']))
                            record['status'] = 'reading'
                        else:
                            if len(response.values) != len(job['names']):
                                raise ValueError('parameter response length differs from request')
                            record.update(status='captured',parameters={key:parameter_record(value)
                                for key,value in zip(job['names'],response.values)},
                                captured_ros_s=node.get_clock().now().nanoseconds*1e-9)
                            del jobs[name]
                except Exception as exc:
                    record.update(status='error',error=str(exc))
                    del jobs[name]
            if jobs:
                pump(min(.02,max(0.,timeout-(time.monotonic()-started))))
        for name in jobs:
            records[name].update(status='timeout',pending_stage=records[name]['status'])
    finally:
        for client in clients:
            node.destroy_client(client)
    failures = [name for name in required_nodes if records[name]['status'] != 'captured']
    snapshot = dict(schema_version=1,role='resolved_ros_parameters_startup_snapshot',
                started_ros_s=started_ros,elapsed_monotonic_s=time.monotonic()-started,
                required_nodes=list(required_nodes),required_complete=not failures,
                incomplete_required_nodes=failures,nodes=records,
                required_timeout_s=timeout,optional_timeout_s=optional_timeout,
                complete=all(record['status']=='captured' for record in records.values()))
    snapshot['motion_parameter_agreement'] = compare_motion_parameters(snapshot)
    return snapshot


def main():
    import argparse
    import json
    from pathlib import Path
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scope',choices=('navigation','transport'),default='navigation')
    args = parser.parse_args()
    required = tuple(name for name in TRANSPORT_REQUIRED_NODES if args.scope=='transport' or
                     name not in ('/g1_arm_command_compositor','/g1_sim_manipulation_io'))
    rclpy.init()
    node = Node('runtime_parameter_capture')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        deadline = time.monotonic()+3.
        while rclpy.ok() and time.monotonic()<deadline:
            present = {namespace.rstrip('/')+'/'+name for name,namespace in node.get_node_names_and_namespaces()}
            if set(required)<=present:
                break
            executor.spin_once(timeout_sec=.02)
        result = capture_parameters(node,lambda seconds:executor.spin_once(timeout_sec=seconds),required_nodes=required)
        args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        passed = result['required_complete'] and result['motion_parameter_agreement']['consistent']
        print(json.dumps(dict(output=str(args.output),required_complete=result['required_complete'],
                              motion_parameters_consistent=result['motion_parameter_agreement']['consistent'])))
        return 0 if passed else 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
