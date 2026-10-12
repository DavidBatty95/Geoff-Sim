"""Read-only RViz overlays for search decisions; never a navigation input."""
import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray


def search_markers(stamp,goals,selected,visited,failed,robot,target,state,planning_ms,objects=(),rooms=(),objective=''):
    output = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    output.markers.append(clear)
    def marker(kind,point,colour,size=.18):
        msg = Marker()
        msg.header.frame_id,msg.header.stamp = 'map',stamp
        msg.ns,msg.id,msg.type = 'object_search',len(output.markers),kind
        msg.pose.orientation.w = 1.
        msg.pose.position.x,msg.pose.position.y,msg.pose.position.z = float(point[0]),float(point[1]),.12
        msg.scale.x=msg.scale.y=msg.scale.z=size
        msg.color.r,msg.color.g,msg.color.b,msg.color.a = colour
        msg.lifetime.sec = 3
        output.markers.append(msg)
        return msg
    for goal in goals[:40]:
        colour = (.1,.9,.3,.65) if goal.kind=='frontier' else (.1,.7,1.,.55)
        marker(Marker.SPHERE,(goal.x,goal.y),colour)
    for point in visited[-40:]:
        marker(Marker.CYLINDER,point,(.55,.55,.55,.65),.25)
    for point in failed[-20:]:
        marker(Marker.CUBE,point,(1.,.1,.1,.8),.22)
    for room in sorted(rooms,key=lambda room:not room['active'])[:32]:
        colour = (.3,1.,.7,.8) if room['active'] else (.7,.45,1.,.5)
        marker(Marker.CYLINDER,room['centroid'],colour,.4)
        label = marker(Marker.TEXT_VIEW_FACING,room['centroid'],colour,.20)
        label.pose.position.z = .65
        label.text = (f'R{room["region"]}: {room["coverage_proxy"]:.0%} view proxy\n'
                      f'{room["inspections"]} views | '+('deferred' if room['deferred'] else 'inspect'))
    for item in objects:
        identifier,x,y,z,age = item[:5]
        approach = item[5] if len(item)>5 else None
        rgb = {'red':(.9,.15,.15),'blue':(.15,.35,1.),'green':(.1,.8,.2)}.get(identifier.rsplit('_',1)[-1],(.8,.8,.8))
        colour = (*rgb,1. if age<=5. else .3)
        item = marker(Marker.CUBE,(x,y),colour,.12 if identifier.startswith('box_') else .36)
        item.pose.position.z = float(z)
        if identifier.startswith('target_'):
            item.scale.z = .02
        label = marker(Marker.TEXT_VIEW_FACING,(x,y),(1.,1.,1.,.9),.16)
        label.pose.position.z = float(z)+.25
        label.text = f'{identifier} ({age:.1f}s old)'
        if approach is not None:
            arrow = marker(Marker.ARROW,(x-.8*math.cos(approach),y-.8*math.sin(approach)),colour)
            arrow.scale.x,arrow.scale.y,arrow.scale.z = .6,.08,.10
            arrow.pose.orientation.z,arrow.pose.orientation.w = math.sin(approach/2),math.cos(approach/2)
            label.text += '\nobserved approach side'
    if selected is not None:
        msg = marker(Marker.ARROW,(selected.x,selected.y),(1.,.55,.05,1.))
        msg.scale.x,msg.scale.y,msg.scale.z = .7,.10,.12
        msg.pose.orientation.z,msg.pose.orientation.w = math.sin(selected.yaw/2),math.cos(selected.yaw/2)
        if robot is not None:
            ray = marker(Marker.LINE_STRIP,robot,(1.,.55,.05,.65),.04)
            ray.pose.position.x=ray.pose.position.y=ray.pose.position.z=0.
            ray.points = [Point(x=float(x),y=float(y),z=.15) for x,y in (robot,(selected.x,selected.y))]
    if robot is not None:
        label = marker(Marker.TEXT_VIEW_FACING,robot,(1.,1.,1.,1.),.22)
        label.pose.position.z = 1.8
        label.text = f'Search: {target or "idle"}\n{state} | {len(goals)} views | {planning_ms:.0f} ms'
        if objective:
            label.text += '\n'+objective
    return output
