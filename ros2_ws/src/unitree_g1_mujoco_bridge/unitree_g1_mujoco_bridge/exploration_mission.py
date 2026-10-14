"""Single-owner search executive layered over the existing transport actions."""
import math
import time
import copy
import json
from collections import deque
from concurrent.futures import ThreadPoolExecutor

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformException

from .exploration_search import ObservedSearchMap,SearchView,view_score,view_novelty
from .transport_actions import ActionTerminationError
from .localization_recovery import BodyInstabilityError
from .transport_people import closest_person_distance
from .search_visualization import search_markers
from .room_search import RoomTopology
from .policy_response import NATIVE_CONTACT_PROFILE


class ExplorationMission:
    def __init__(self,mission,max_views=20):
        self.mission = mission
        self.objective = getattr(mission.args,'objective','')
        self.max_views = max_views
        self.turn_rate = .30 if getattr(mission.args,'response_profile','legacy')==NATIVE_CONTACT_PROFILE else .18
        self.map = None
        self.visits,self.failures = {},[]
        self.target,self.selected = None,None
        self.last_target = None
        self.state,self.generation = 'idle',0
        self.candidates,self.robot = [],None
        self.pool = ThreadPoolExecutor(max_workers=1,thread_name_prefix='search-views')
        self.pending,self.prepared = None,None
        self.planning_ms = 0.
        self.topology,self.room_anchor,self.inspection_heading = None,None,None
        self.inspection_kind = None
        self.rooms,self.coverage,self.inspections,self.coverage_stamp = [],{},{},{}
        self.inspection_coverage = deque(maxlen=160)
        self.geometry_cache = None
        self.memory_transform = None
        self.coverage_transform = None

        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.subscription = mission.create_subscription(OccupancyGrid,'/map',self._map,qos)
        self.markers = mission.create_publisher(MarkerArray,'/exploration/search_markers',1)
        self.status = mission.create_publisher(String,'/exploration/search_status',1)
        self.timer = mission.create_timer(.5,self._prepare)
        self.visual_timer = mission.create_timer(.5,self._visualize)

    def close(self):
        self.timer.cancel()
        self.visual_timer.cancel()
        self.pool.shutdown(wait=True,cancel_futures=True)

    def _visualize(self):
        age = None if self.prepared is None else time.monotonic()-self.prepared[1]
        visible = self.candidates if age is not None and age<=2. else []
        self.status.publish(String(data=json.dumps(dict(target=self.target,state=self.state,
            last_target=self.last_target,
            objective=self.objective,
             candidate_count=len(visible),planning_ms=self.planning_ms,plan_age_s=age,
            rooms=self.rooms,
            selected=None if self.selected is None else [self.selected.x,self.selected.y,self.selected.yaw],
            inspected=sum(map(len,self.visits.values())),failed=len(self.failures)))))
        if self.markers.get_subscription_count():
            objects = []
            try:
                tf = self._transform('map','odom')
                for identifier,item in self.mission.search_hints.items():
                    x,y = self._point(item['position'],tf)
                    approach = item['approach_yaw']+tf[2] if 'approach_yaw' in item else None
                    objects.append((identifier,x,y,item['position'][2],self.mission.seconds()-item['observed_stamp'],approach))
            except TransformException:
                pass
            self.markers.publish(search_markers(self.mission.get_clock().now().to_msg(),visible,self.selected,
                [point for points in self.visits.values() for point in points],[p for p,_ in self.failures],
                self.robot,self.target or self.last_target,self.state,self.planning_ms,objects,self.rooms,self.objective))

    def _key(self):
        return self.target,self.generation,self.mission.object_may_be_held

    def _compute(self,message,robot,visited,failed,people,radius,heading,surfaces=()):
        start = time.monotonic()
        info = message.info
        key = (radius,info.width,info.height,info.resolution,str(info.origin),bytes((value+1 for value in message.data)))
        if self.geometry_cache is None or self.geometry_cache[0]!=key:
            grid = ObservedSearchMap(message,radius,self.turn_rate)
            topology = RoomTopology(grid)
            if self.geometry_cache is not None:
                topology.reconcile(self.geometry_cache[2])
            self.geometry_cache = key,grid,topology
        else:
            _,grid,topology = self.geometry_cache
        goals = grid.viewpoints(robot,visited,failed,people,heading=heading)
        goals += grid.support_viewpoints(robot,surfaces,visited,people,heading) if surfaces else []
        goals.sort(key=lambda view:-view.score)
        x,y = grid.cell(robot)
        root_clear = 0<=x<grid.width and 0<=y<grid.height and bool(grid.free[y,x])
        return goals,root_clear,(time.monotonic()-start)*1000.,topology

    def _prepare(self):
        node = self.mission
        if self.pending is not None and self.pending[1].done():
            key,future,submitted = self.pending
            self.pending = None
            try:
                goals,root_clear,elapsed,topology = future.result()
            except ValueError:
                self.state = 'invalid_map'
            else:
                if key==self._key():
                    self.prepared = key,submitted,goals,root_clear
                    self.candidates,self.planning_ms = goals,elapsed
                    self.topology = topology
        if self.target is None or self.pending is not None:
            return
        if self.prepared is not None and self.prepared[0]==self._key() and time.monotonic()-self.prepared[1]<1.:
            return
        if not self._fresh_map() or node.pose is None or not 0.<=node.seconds()-node.pose_stamp<=.5:
            return
        try:
            tf = self._transform('map','odom')
        except TransformException:
            return
        self._sync_memory(tf)
        people = self._people(tf)
        if people is None:
            return
        self.robot = self._point(node.pose[:2],tf)
        failed = [point for point,stamp in self.failures if time.monotonic()-stamp<60.]
        key = self._key()
        submitted = time.monotonic()
        # Detached inputs; one outstanding computation, no ROS operations in worker.
        future = self.pool.submit(self._compute,copy.deepcopy(self.map),self.robot,
            tuple(self.inspections.get(self.target,())),tuple(failed),tuple(people),.65 if key[2] else .55,node.pose[2]+tf[2],
            tuple((*self._point(item['position'],tf),.2 if any(
                identifier!=self.target and 0.<=node.seconds()-hint['observed_stamp']<300. and
                math.dist(item['position'][:2],hint['position'][:2])<.85
                for identifier,hint in node.search_hints.items()) else 1.)
                for item in getattr(node,'support_memory',[])
                  if 0.<=node.seconds()-item['observed_stamp']<120.))
        self.pending = key,future,submitted

    def _ready(self):
        self._prepare()
        return self.prepared is not None and self.prepared[0]==self._key() and time.monotonic()-self.prepared[1]<=2.

    def _revalidated(self,visited):
        node = self.mission
        if not self._fresh_map() or not 0.<=node.seconds()-node.pose_stamp<=.5:
            return []
        tf = self._transform('map','odom')
        robot = self._point(node.pose[:2],tf)
        people = self._people(tf)
        if people is None:
            return []
        grid = ObservedSearchMap(self.map,.65 if node.object_may_be_held else .55)
        reachable = grid.reachable(robot,80.)
        result = []
        for goal in self.prepared[2]:
            point = goal.x,goal.y
            x,y = grid.cell(point)
            if not (0<=x<grid.width and 0<=y<grid.height and reachable[y,x]>=0):
                continue
            novelty = view_novelty(point,goal.yaw,visited)
            if (math.dist(robot,point)<.6 and goal.kind!='heading_inspection') or novelty<1.1:
                continue
            if any(closest_person_distance((px,py),(vx,vy),point,0.)<radius+grid.radius+.4 for px,py,vx,vy,radius in people):
                continue
            # The worker prepared these during travel, before this inspection
            # was recorded. Refresh novelty and travel cost as well as admission;
            # otherwise stale high novelty repeatedly selects nearby floor.
            cost = float(reachable[y,x])*grid.resolution
            yaw = goal.yaw if goal.kind in ('support_inspection','known_support_inspection','heading_inspection') else math.atan2(point[1]-robot[1],point[0]-robot[0])
            turn = abs(math.atan2(math.sin(yaw-node.pose[2]-tf[2]),math.cos(yaw-node.pose[2]-tf[2])))
            if goal.kind in ('support_inspection','known_support_inspection'):
                bearing = math.atan2(point[1]-robot[1],point[0]-robot[0])
                turn = (abs(math.atan2(math.sin(bearing-node.pose[2]-tf[2]),math.cos(bearing-node.pose[2]-tf[2])))+
                        abs(math.atan2(math.sin(yaw-bearing),math.cos(yaw-bearing))))
            risk = sum(1./max(.2,closest_person_distance((px,py),(vx,vy),point,0.)-radius)
                       for px,py,vx,vy,radius in people)
            gain = 16 if goal.kind=='support_inspection' else 3 if goal.kind=='known_support_inspection' else goal.information
            result.append(SearchView(*point,yaw,view_score(cost,gain,novelty,turn,risk,getattr(self,'turn_rate',.18)),
                                     cost,goal.information,goal.kind))
        return sorted(result,key=lambda goal:(-goal.score,goal.path_length,goal.x,goal.y))

    def _map(self,message):
        if message.header.frame_id=='map':
            self.map = message

    def _fresh_map(self):
        if self.map is None:
            return False
        stamp = self.map.header.stamp.sec+self.map.header.stamp.nanosec*1e-9
        return 0.<=self.mission.seconds()-stamp<=10.

    def _transform(self,target,source):
        tf = self.mission.tf.lookup_transform(target,source,rclpy.time.Time()).transform
        q = tf.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return tf.translation.x,tf.translation.y,yaw

    @staticmethod
    def _point(point,tf):
        x,y,yaw = tf
        c,s = math.cos(yaw),math.sin(yaw)
        return x+c*point[0]-s*point[1],y+s*point[0]+c*point[1]

    def _sync_memory(self,tf):
        """Carry soft view memory through SLAM frame corrections, not region IDs."""
        previous = self.memory_transform
        self.memory_transform = tf
        if previous is None or previous==tf:
            if self.coverage_transform is None:
                self.coverage_transform = tf
            return
        c,s = math.cos(previous[2]),math.sin(previous[2])
        inverse = (-c*previous[0]-s*previous[1],s*previous[0]-c*previous[1],-previous[2])
        def moved(point):
            return self._point(self._point(point,inverse),tf)
        yaw_delta = tf[2]-previous[2]
        for points in self.visits.values():
            points[:] = [moved(p) for p in points]
        for points in self.inspections.values():
            points[:] = [(*moved(p[:2]),p[2]+yaw_delta) for p in points]
        # Coverage cells are a lossy proxy. Reproject only for a material frame
        # correction; repeated sub-cell rounding would otherwise diffuse memory.
        if math.dist(previous[:2],tf[:2])>.25 or abs(yaw_delta)>.1:
            # A large pose-graph correction may be non-rigid. Discard the
            # coarse negative-evidence proxy instead of claiming unseen space
            # has been inspected. Positive object memory remains in odom.
            self.coverage = {key:set() for key in self.coverage}
            if hasattr(self,'inspection_coverage'):
                self.inspection_coverage.clear()
            self.coverage_transform = tf
            self.generation += 1
            self.prepared = None
            self.mission.event(getattr(self.mission,'phase','search_frame_correction'),search_frame_correction=True,
                               translation_m=math.dist(previous[:2],tf[:2]),yaw_rad=yaw_delta)
        self.failures = [(moved(p),stamp) for p,stamp in self.failures]
        if self.room_anchor is not None:
            self.room_anchor = moved(self.room_anchor)

    def _people(self,tf):
        node = self.mission
        if node.people is None or node.people.header.frame_id!='odom':
            return None
        stamp = node.people.header.stamp.sec+node.people.header.stamp.nanosec*1e-9
        age = node.seconds()-stamp
        if not 0.<=age<=.8:
            return None
        people = []
        c,s = math.cos(tf[2]),math.sin(tf[2])
        for person in node.people.persons:
            p,v = person.pose.pose.position,person.velocity.twist.linear
            if not math.isfinite(person.body_radius) or person.body_radius<0.:
                return None
            x,y = self._point((p.x+age*v.x,p.y+age*v.y),tf)
            people.append((x,y,c*v.x-s*v.y,s*v.x+c*v.y,max(.4,float(person.body_radius))))
        return people

    def discover(self,identifier):
        self.target,self.selected = identifier,None
        self.last_target = identifier
        self.room_anchor,self.inspection_heading = None,None
        self.inspection_kind = None
        # Every registered image processes all task colours. Reuse its soft
        # visibility evidence across box/goal searches, but never as discovery.
        shared_seen = set().union(*self.coverage.values()) if self.coverage else set()
        shared_views = []
        for views in self.inspections.values():
            for view in views:
                if view not in shared_views:
                    shared_views.append(view)
        self.coverage.setdefault(identifier,set()).update(shared_seen)
        self.inspections.setdefault(identifier,list(shared_views))
        self.generation += 1
        self.prepared = None
        try:
            observed = self._discover(identifier)
            self.state = 'found'
            return observed
        except BaseException:
            self.state = 'search_failed'
            raise
        finally:
            self.target,self.selected = None,None
            self.generation += 1
            self._visualize()

    def inspect_support(self,identifier,item):
        """Acquire a support-normal approach before committing to a close dock."""
        node = self.mission
        self.last_target = identifier
        visited = []
        for attempt in range(3):
            latest = node.observe(identifier,5.)
            if latest is not None and 'approach_yaw' in latest:
                return latest
            if latest is not None:
                item = latest
            node.wait(lambda:self._fresh_map() and node.pose is not None and
                      0.<=node.seconds()-node.pose_stamp<=.5,5.,'fresh support inspection inputs unavailable')
            tf = self._transform('map','odom')
            people = self._people(tf)
            if people is None:
                node.spin(.2)
                continue
            grid = ObservedSearchMap(self.map,.65 if node.object_may_be_held else .55)
            robot,target = self._point(node.pose[:2],tf),self._point(item['position'],tf)
            views = [view for view in grid.semantic_viewpoints(robot,target,people)
                     if all(math.dist((view.x,view.y),point)>.6 for point in visited)]
            if not views:
                if self.recover_search_reserve():
                    continue
                raise RuntimeError('no observed-clear support inspection viewpoint')
            view = views[0]
            visited.append((view.x,view.y))
            inverse = self._transform('odom','map')
            x,y = self._point((view.x,view.y),inverse)
            node.event('inspect_support',object_id=identifier,attempt=attempt,source='confirmed_rgbd_and_observed_map')
            self.navigate_observed((x,y,view.yaw+inverse[2]),phase='delivery_route' if identifier.startswith('target_') else 'pickup_route')
            bearing = math.atan2(item['position'][1]-node.pose[1],item['position'][0]-node.pose[0])
            node.rotate(bearing)
            settled = node.seconds()
            node.wait(lambda:node.last_image_stamp is not None and node.last_image_stamp>settled,
                      5.,'no fresh support inspection image')
            first = node.last_image_stamp
            node.wait(lambda:node.last_image_stamp>first,3.,'no second support inspection image')
        latest = node.observe(identifier,5.)
        if latest is not None and 'approach_yaw' in latest:
            return latest
        raise RuntimeError(f'support edge not confirmed for {identifier}')

    def recover_search_reserve(self,timeout=8.):
        """Recover from coasting into the extra search inflation, not into a wall."""
        node = self.mission
        if not self._fresh_map() or node.pose is None or not 0.<=node.seconds()-node.pose_stamp<=.5:
            return False
        tf = self._transform('map','odom')
        people = self._people(tf)
        if people is None:
            return False
        radius = .65 if node.object_may_be_held else .55
        physical_radius = .54 if node.object_may_be_held else .38
        grid = ObservedSearchMap(self.map,radius)
        robot = self._point(node.pose[:2],tf)
        point = grid.clearance_recovery_view(robot,node.pose[2]+tf[2],physical_radius,people)
        if point is None:
            return False
        x,y = self._point(point,self._transform('odom','map'))
        node.event('search_reserve_recovery',goal_odom=[x,y,node.pose[2]],
                   physical_radius_m=physical_radius,search_radius_m=radius)
        try:
            node.set_goal((x,y,node.pose[2]),'reserve_recovery')
            def recovered():
                if not self._fresh_map() or not 0.<=node.seconds()-node.pose_stamp<=.5:
                    return False
                current = self._point(node.pose[:2],self._transform('map','odom'))
                return math.dist(current,point)<.08 and ObservedSearchMap(self.map,radius).segment_free(current,current)
            node.wait(recovered,timeout,'search clearance reserve did not recover')
        finally:
            node.stop_intent()
        self.generation += 1
        self.prepared = None
        return True

    def survey_before_pickup(self,box,destination):
        """Compare sensed empty/loaded travel when a confirmed goal hint exists.

        Unknown destinations retain empty-hand survey. An old sighting is only
        a route hint; loaded delivery still requires fresh RGB-D confirmation.
        """
        node = self.mission
        hint = node.search_hints.get(destination)
        if hint is None or not self._fresh_map() or not 0.<=node.seconds()-hint['observed_stamp']<=90.:
            return True
        try:
            tf = self._transform('map','odom')
            people = self._people(tf)
        except TransformException:
            return True
        if people is None:
            return True
        grid = ObservedSearchMap(self.map,.65)
        robot = self._point(node.pose[:2],tf)
        pickup = grid.semantic_viewpoints(robot,self._point(box['position'],tf),people)
        delivery = grid.semantic_viewpoints(robot,self._point(hint['position'],tf),people)
        if not pickup or not delivery:
            return True
        source,target = (pickup[0].x,pickup[0].y),(delivery[0].x,delivery[0].y)
        route = grid.route(source,target)
        if not route:
            return True
        length = sum(math.dist(a,b) for a,b in zip([source]+route,route))
        hint_age = node.seconds()-hint['observed_stamp']
        pick_first = pickup[0].path_length/.45+length/.35+5.+.1*hint_age
        survey_first = delivery[0].path_length/.45+length/.45+length/.35+5.
        node.event('task_order',pickup_first_s=pick_first,survey_first_s=survey_first,
                   hint_age_s=hint_age,source='observed_routes_and_confirmed_rgbd_memory')
        return pick_first>survey_first

    def navigate_observed(self,goal,timeout=90.,stop_when_observed=None,phase='pickup_route'):
        """Replan observed corner checkpoints instead of chasing a blocked bearing."""
        node = self.mission
        deadline = time.monotonic()+timeout
        visited = []
        for attempt in range(8):
            if stop_when_observed is not None and node.observe(stop_when_observed,5.) is not None:
                return dict(node.observe(stop_when_observed,5.))
            node.wait(lambda:self._fresh_map() and node.pose is not None and
                      0.<=node.seconds()-node.pose_stamp<=.5,5.,'fresh pickup route inputs unavailable')
            tf = self._transform('map','odom')
            start,target = self._point(node.pose[:2],tf),self._point(goal[:2],tf)
            message = copy.deepcopy(self.map)
            planned = time.monotonic()
            radius = .65 if node.object_may_be_held else .55
            route,complete = node.future(self.pool.submit(lambda:ObservedSearchMap(message,radius).route_towards(start,target,tuple(visited))))
            if time.monotonic()-planned>2. or not self._fresh_map():
                continue
            if not route:
                grid = ObservedSearchMap(message,radius)
                node.event(phase+'_blocked',start_clear=grid.segment_free(start,start),
                            goal_clear=grid.segment_free(target,target),radius_m=radius)
                if self.recover_search_reserve(timeout=min(8.,max(0.,deadline-time.monotonic()))):
                    continue
                raise RuntimeError('no observed-clear route')
            # Revalidate the first segment from the latest sensed pose. People
            # still arbitrate every command through the existing navigation chain.
            tf = self._transform('map','odom')
            start,target = self._point(node.pose[:2],tf),self._point(goal[:2],tf)
            if (complete and math.dist(route[-1],target)>.1) or not 0.<=node.seconds()-node.pose_stamp<=.5:
                continue
            if not ObservedSearchMap(self.map,radius).segment_free(start,route[0]):
                continue
            inverse = self._transform('odom','map')
            # Nav2 already follows the full observed route. Sending each visible
            # corner as a separate action unnecessarily stopped/restarted gait.
            # Partial routes still advance to their observed boundary and replan.
            checkpoint = route[-1] if complete else route[0]
            x,y = self._point(checkpoint,inverse)
            final = complete
            yaw = goal[2] if final else math.atan2(y-node.pose[1],x-node.pose[0])
            self.robot,self.state = start,phase
            self.selected = SearchView(*checkpoint,yaw+tf[2],0.,0.,0,'pickup_route')
            self._visualize()
            node.event(phase,checkpoint_odom=[x,y,yaw],remaining_checkpoints=len(route),
                       complete_route=complete,planning_ms=(time.monotonic()-planned)*1000.,source='observed_slam_map')
            remaining = deadline-time.monotonic()
            if remaining<=0.:
                break
            discovery = node.navigate((x,y,yaw),timeout=remaining if complete else min(45.,remaining),
                                      settle_time=.1,stop_when_observed=stop_when_observed)
            if discovery is not None:
                return discovery
            if stop_when_observed is not None and node.observe(stop_when_observed,5.) is not None:
                return dict(node.observe(stop_when_observed,5.))
            if final:
                self.selected,self.state = None,phase+'_ready'
                return
            if len(route)==1:
                visited.append(route[0])
                continue
            # A route corner needs a clear continuation, not manipulation-level
            # centimetre settling. Advance only until the next segment clears
            # the inflated observed footprint; keep precision docking for grasp.
            def continuation_clear():
                if stop_when_observed is not None and node.observe(stop_when_observed,5.) is not None:
                    return True
                if not self._fresh_map() or not 0.<=node.seconds()-node.pose_stamp<=.5:
                    return False
                current = self._point(node.pose[:2],self._transform('map','odom'))
                grid = ObservedSearchMap(self.map,radius)
                # A map update may invalidate the old continuation. Reaching
                # the corner permits a fresh route computation, never blind
                # dispatch of the formerly clear next segment.
                return (grid.segment_free(current,route[1]) or
                        (math.dist(current,route[0])<.15 and grid.segment_free(current,current)))
            try:
                if not continuation_clear():
                    node.set_goal((x,y,yaw),'dock')
                    node.wait(continuation_clear,min(15.,max(0.,deadline-time.monotonic())),
                              'observed route corner did not clear')
            finally:
                node.stop_intent()
            node.spin(.1)
        raise RuntimeError('observed route budget exhausted')

    def _discover(self,identifier):
        node = self.mission
        visited = self.visits.setdefault(identifier,[])
        self._prepare()
        node.event('search_request',object_id=identifier,carrying=node.object_may_be_held,
                   initially_visible=node.observe(identifier,5.) is not None)
        hint_tried = False
        blocked_scans = 0
        stalled_turns,stalled_at = 0,None
        for attempt in range(self.max_views):
            if stalled_at is not None and math.dist(node.pose[:2],stalled_at)>.6:
                stalled_turns,stalled_at = 0,None
            observed = node.observe(identifier,5.)
            if observed is not None:
                node.event('search_found',object_id=identifier,observed_stamp=observed['observed_stamp'],view_count=len(visited))
                return observed
            node.event('search_scan',object_id=identifier,attempt=attempt,carrying=node.object_may_be_held)
            self.state = 'inspect_current_view'
            # Inspect the current viewpoint before choosing another. Negative
            # evidence requires images acquired after each turn has settled.
            start_yaw = node.pose[2]
            # The region planner aims at uncovered area rather than rotating
            # an arbitrary quarter-turn after each short translation.
            offset = 0. if self.inspection_heading is None else math.atan2(
                math.sin(self.inspection_heading-start_yaw),math.cos(self.inspection_heading-start_yaw))
            # Route visualization replaces/clears self.selected while travelling;
            # preserve the camera task independently through navigation cleanup.
            semantic = self.inspection_kind in ('support_inspection','known_support_inspection','semantic_reobserve')
            offset = offset if semantic else max(-math.pi/3,min(math.pi/3,offset))
            if stalled_turns>=2:
                offset = 0.
            self.inspection_heading = None
            self.inspection_kind = None
            for offset in ((offset if abs(offset)>.25 else 0.),):
                if offset:
                    try:
                        discovery = node.rotate(math.atan2(math.sin(start_yaw+offset),math.cos(start_yaw+offset)),
                                    timeout=30. if semantic else 12.,settle_before_turn=True,stop_when_observed=identifier)
                        if discovery is not None:
                            node.event('search_found',object_id=identifier,observed_stamp=discovery['observed_stamp'],
                                       view_count=len(visited),source='confirmed_acquisition_before_turn_braking')
                            return discovery
                        stalled_turns,stalled_at = 0,None
                    except RuntimeError as exc:
                        if str(exc) not in ('search heading not reached','search heading stalled','base did not settle before inspection turn',
                                            'search turn displaced beyond viewpoint reserve') or time.monotonic()-node.started>=node.args.duration:
                            raise
                        # An exploratory camera view may be partial. Its actual
                        # sensed heading determines coverage after fresh images.
                        node.event('search_partial_turn',actual_yaw=node.pose[2],reason=str(exc))
                        stalled_turns += 1
                        if stalled_at is None:
                            stalled_at = node.pose[:2]
                        node.spin(.5)
                settled = node.seconds()
                node.wait(lambda: node.last_image_stamp is not None and node.last_image_stamp>settled,
                          5.,'no fresh RGB-D acquisition for search inspection')
                first_image = node.last_image_stamp
                node.wait(lambda:node.observe(identifier,5.) is not None or node.last_image_stamp>first_image,
                          3.,'no second RGB-D acquisition for search inspection')
                observed = node.observe(identifier,5.)
                if observed is not None:
                    node.event('search_found',object_id=identifier,observed_stamp=observed['observed_stamp'],view_count=len(visited))
                    return observed
            node.wait(self._fresh_map,15.,'fresh SLAM map unavailable for search')
            node.wait(lambda: node.pose is not None and 0.<=node.seconds()-node.pose_stamp<=.5,
                      5.,'fresh pose unavailable for search')
            tf = self._transform('map','odom')
            robot = self._point(node.pose[:2],tf)
            visited.append(robot)
            self.robot = robot
            node.event('search_inspected',object_id=identifier,viewpoint_map=list(robot),image_stamp=node.last_image_stamp)
            self.state = 'select_viewpoint'
            selection_start = time.monotonic()
            node.wait(self._ready,5.,'no fresh exploration plan available')
            if self.topology is not None:
                tf = self._transform('map','odom')
                robot = self._point(node.pose[:2],tf)
                image_pose = getattr(node,'last_image_pose',None)
                acquisition_position = self._point(image_pose[:2],tf) if image_pose is not None else robot
                yaw = image_pose[2]+tf[2] if image_pose is not None else node.pose[2]+tf[2]
                seen = self.coverage.setdefault(identifier,set())
                inspections = self.inspections.setdefault(identifier,[])
                if node.last_image_stamp!=self.coverage_stamp.get(identifier):
                    self.coverage_stamp[identifier] = node.last_image_stamp
                    visible_keys = self.topology.visible(acquisition_position,yaw,ObservedSearchMap(self.map,.55),
                                                         getattr(node,'last_image_hfov',math.pi/3))
                    seen.update(visible_keys)
                    if not any(math.dist(acquisition_position,view[:2])<.6 and
                               abs(math.atan2(math.sin(yaw-view[2]),math.cos(yaw-view[2])))<.35 for view in inspections):
                        inspections.append((*acquisition_position,yaw))
                        self.inspection_coverage.append(frozenset(visible_keys))
                candidates = self._revalidated(inspections or visited)
                if stalled_turns>=2:
                    candidates = [view for view in candidates if view.kind!='heading_inspection']
                # A local camera sweep is cheaper than leaving an already
                # observed room and returning, especially after an immediate
                # box discovery. Consider the uninspected adjacent camera cone
                # before an unnecessary translation during the first views.
                local_views = [view for view in inspections if math.dist(view[:2],robot)<.8]
                root_grid = ObservedSearchMap(self.map,.65 if node.object_may_be_held else .55)
                if len(local_views)<3 and stalled_turns<2 and root_grid.segment_free(robot,robot):
                    for offset in (-math.pi/3,math.pi/3):
                        direction = node.pose[2]+tf[2]+offset
                        novelty = view_novelty(robot,direction,inspections)
                        if novelty>1.1:
                            visible = self.topology.visible(robot,direction,ObservedSearchMap(self.map,.55),
                                                            getattr(node,'last_image_hfov',math.pi/3))
                            gain = len(visible-seen)
                            if gain>=5:
                                candidates.append(SearchView(*robot,direction,.06*gain,0.,0,'heading_inspection'))
                candidates,self.rooms = self.topology.decide(candidates,robot,seen,inspections,self.room_anchor,
                                                           heading=node.pose[2]+tf[2],hfov=getattr(node,'last_image_hfov',math.pi/3),
                                                           turn_rate=self.turn_rate,inspection_coverage=self.inspection_coverage)
                node.event('search_room_decision',object_id=identifier,regions=len(self.rooms),
                           selected=next((room for room in self.rooms if room['active']),None),
                            coverage_source='fresh_rgbd_inspection_with_observed_map_visibility_proxy')
            else:
                candidates = self._revalidated(visited)
            hint = node.search_hints.get(identifier)
            if (hint is not None and not hint_tried and self._fresh_map() and
                    0.<=node.seconds()-node.pose_stamp<=.5):
                hint_tried = True
                tf = self._transform('map','odom')
                robot = self._point(node.pose[:2],tf)
                people = self._people(tf)
                if people is not None:
                    grid = ObservedSearchMap(self.map,.65 if node.object_may_be_held else .55)
                    memory_views = grid.semantic_viewpoints(robot,self._point(hint['position'],tf),people)
                    if memory_views:
                        candidates = memory_views
                        node.event('search_memory',object_id=identifier,hint_age_s=node.seconds()-hint['observed_stamp'],
                                   source='previous_confirmed_rgbd')
            if not candidates:
                self.state = 'blocked_start' if self.prepared is None or not self.prepared[3] else 'no_reachable_unvisited_view'
                if self.state=='blocked_start' and self.recover_search_reserve():
                    continue
                blocked_scans += 1
                if blocked_scans>=3:
                    raise RuntimeError('search has no reachable useful viewpoint after three fresh inspections')
                self.generation += 1
                self._visualize()
                node.stop_intent()
                self.inspection_heading = node.pose[2]+math.pi/2 if stalled_turns<2 else None
                node.spin(.5)
                continue  # No frontier is not proof that the requested object exists nowhere.
            blocked_scans = 0
            selected = candidates[0]
            self.room_anchor = selected.x,selected.y
            self.candidates = candidates
            self.selected,self.state = selected,'navigate_to_view'
            self._visualize()
            node.event('search_viewpoint',object_id=identifier,viewpoint_map=[selected.x,selected.y],
                       utility=selected.score,path_length_m=selected.path_length,information=selected.information,
                       source='observed_slam_map',kind=selected.kind,planning_ms=self.planning_ms,
                       selection_wait_ms=(time.monotonic()-selection_start)*1000.)
            inverse = self._transform('odom','map')
            x,y = self._point((selected.x,selected.y),inverse)
            self.inspection_heading = selected.yaw+inverse[2]
            self.inspection_kind = selected.kind
            if selected.kind=='heading_inspection':
                continue  # Inspect another camera cone without a redundant Nav2 action.
            try:
                discovery = self.navigate_observed((x,y,selected.yaw+inverse[2]),
                    timeout=min(120.,max(45.,selected.path_length/.35+15.)),stop_when_observed=identifier,phase='search_route')
                if discovery is not None:
                    node.event('search_found',object_id=identifier,observed_stamp=discovery['observed_stamp'],
                               view_count=len(visited),source='confirmed_acquisition_before_navigation_termination')
                    return discovery
            except (ActionTerminationError,BodyInstabilityError):
                raise  # Never dispatch another goal if previous ownership is uncertain.
            except RuntimeError as exc:
                if time.monotonic()-node.started>=node.args.duration:
                    raise
                self.failures.append(((selected.x,selected.y),time.monotonic()))
                self.generation += 1
                node.event('search_viewpoint_failed',object_id=identifier,error=str(exc))
            observed = node.observe(identifier,5.)
            if observed is not None:
                node.event('search_found',object_id=identifier,observed_stamp=observed['observed_stamp'],view_count=len(visited))
                return observed
        raise RuntimeError(f'search budget exhausted without a confirmed {identifier}')
