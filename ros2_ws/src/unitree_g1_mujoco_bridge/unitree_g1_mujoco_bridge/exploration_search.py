"""Observed-map search viewpoints; no simulator/layout inputs or motor commands.

Frontier information, camera-view novelty and travel cost are heuristic utilities.
This is not an implementation of SafeEstMDP's probabilistic safety guarantees.
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .transport_people import closest_person_distance


@dataclass(frozen=True)
class SearchView:
    x: float
    y: float
    yaw: float
    score: float
    path_length: float
    information: int
    kind: str


def view_score(cost,gain,novelty,turn,risk,turn_rate=.18):
    # Time proxy includes slow measured humanoid yaw response and acquisition.
    return (1.+.35*gain+min(9.,novelty*novelty))/(1.+cost/.45+turn/max(.05,turn_rate))-.25*risk


def view_novelty(point,yaw,visited):
    """Position-only legacy visits remain supported; oriented views can revisit."""
    distances = []
    for view in visited:
        distance = math.dist(point,view[:2])
        if len(view)>2:
            heading = abs(math.atan2(math.sin(yaw-view[2]),math.cos(yaw-view[2])))
            distance = math.hypot(distance,heading*1.5)
        distances.append(distance)
    return min(distances,default=3.)


class ObservedSearchMap:
    def __init__(self,message,radius=.55,turn_rate=.18):
        self.message = message
        self.turn_rate = turn_rate
        info = message.info
        self.resolution = float(info.resolution)
        self.radius = radius
        self._reach_cache = None
        self.width,self.height = int(info.width),int(info.height)
        if (self.resolution<=0. or not math.isfinite(self.resolution) or
                self.width<=0 or self.height<=0 or self.width*self.height>500000 or
                len(message.data)!=self.width*self.height or not math.isfinite(radius) or radius<0.):
            raise ValueError('invalid or oversized exploration map')
        p,q = info.origin.position,info.origin.orientation
        self.origin = (p.x,p.y)
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if not all(map(math.isfinite,(*self.origin,self.yaw))):
            raise ValueError('nonfinite exploration map origin')
        self.data = np.asarray(message.data,dtype=np.int16).reshape(self.height,self.width)
        free = (self.data>=0)&(self.data<20)
        # A square reserve is conservative for the robot's circular footprint.
        # Unknown cells and the outside of the map are not traversable.
        cells = math.ceil(radius/self.resolution)
        size = 2*cells+1
        if size>min(self.width,self.height):
            self.free = np.zeros_like(free)
            return
        padded = np.pad(free.astype(np.int32),cells)
        integral = np.pad(padded,((1,0),(1,0))).cumsum(0).cumsum(1)
        counts = integral[size:,size:]-integral[:-size,size:]-integral[size:,:-size]+integral[:-size,:-size]
        self.free = counts==size*size

    def cell(self,point):
        dx,dy = point[0]-self.origin[0],point[1]-self.origin[1]
        c,s = math.cos(self.yaw),math.sin(self.yaw)
        return math.floor((c*dx+s*dy)/self.resolution),math.floor((-s*dx+c*dy)/self.resolution)

    def footprint_evidence(self,point):
        """Explain the conservative square reserve; never clear map evidence."""
        x,y = self.cell(point)
        cells = math.ceil(self.radius/self.resolution)
        inside = 0<=x<self.width and 0<=y<self.height
        x0,x1 = max(0,x-cells),min(self.width,x+cells+1)
        y0,y1 = max(0,y-cells),min(self.height,y+cells+1)
        values = self.data[y0:y1,x0:x1] if x1>x0 and y1>y0 else np.empty(0)
        occupied = int(np.count_nonzero(values>=20))
        unknown = int(np.count_nonzero(values<0))
        outside = (2*cells+1)**2-values.size
        reason = ('map_boundary' if outside else 'occupied_square_reserve' if occupied else
                  'unknown_square_reserve' if unknown else 'clear')
        return dict(clear=bool(inside and self.free[y,x]),reason=reason,
                    radius_m=self.radius,cell=[x,y],occupied_cells=occupied,
                    unknown_cells=unknown,outside_cells=int(outside))

    def world(self,x,y):
        c,s = math.cos(self.yaw),math.sin(self.yaw)
        px,py = (x+.5)*self.resolution,(y+.5)*self.resolution
        return self.origin[0]+c*px-s*py,self.origin[1]+s*px+c*py

    def reachable(self,robot,max_distance):
        x,y = self.cell(robot)
        key = x,y,max_distance
        if self._reach_cache is not None and self._reach_cache[0]==key:
            return self._reach_cache[1]
        width,height = self.width,self.height
        distance = [[-1]*width for _ in range(height)]
        if not (0<=x<self.width and 0<=y<self.height and self.free[y,x]):
            return np.asarray(distance,dtype=np.int32)
        # Python scalar indexing avoids repeated NumPy scalar dispatch in the
        # flood-fill inner loop; conversion at the boundary remains bounded.
        free = self.free.tolist()
        distance[y][x] = 0
        queue = deque([(x,y)])
        limit = math.ceil(max_distance/self.resolution)
        while queue:
            x,y = queue.popleft()
            current = distance[y][x]
            if current>=limit:
                continue
            for nx,ny in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
                if 0<=nx<width and 0<=ny<height and free[ny][nx] and distance[ny][nx]<0:
                    distance[ny][nx] = current+1
                    queue.append((nx,ny))
        result = np.asarray(distance,dtype=np.int32)
        result.flags.writeable = False
        self._reach_cache = key,result
        return result

    def information(self,x,y,view_range=3.):
        return self.information_batch([(x,y)],view_range)[0]

    def visible_points(self,origin,points):
        """Optical LOS on occupancy, separate from inflated base traversability.

        Unknown cells occlude; a camera ray does not require a robot-sized gap.
        Batch rays to avoid one Python/NumPy allocation per visible floor cell.
        """
        points = np.asarray(points,dtype=float).reshape(-1,2)
        if not len(points):
            return np.zeros(0,dtype=bool)
        start = np.asarray(self.cell(origin),dtype=float)+.5
        c,s = math.cos(self.yaw),math.sin(self.yaw)
        delta = points-np.asarray(self.origin)
        ends = np.column_stack((c*delta[:,0]+s*delta[:,1],-s*delta[:,0]+c*delta[:,1]))/self.resolution
        count = max(2,int(np.ceil(np.max(np.linalg.norm(ends-start,axis=1))*2))+1)
        rays = np.floor(start+(ends[:,None,:]-start)*np.linspace(0.,1.,count)[None,:,None]).astype(int)
        xs,ys = rays[:,:,0],rays[:,:,1]
        inside = (xs>=0)&(xs<self.width)&(ys>=0)&(ys<self.height)
        cells = self.data[np.clip(ys,0,self.height-1),np.clip(xs,0,self.width-1)]
        return np.all(inside&(cells>=0)&(cells<20),axis=1)

    def segment_free(self,start,end):
        """Conservative supercover sampling on the already inflated grid."""
        if not all(math.isfinite(value) for point in (start,end) for value in point):
            return False
        for point in (start,end):
            x,y = self.cell(point)
            if not (0<=x<self.width and 0<=y<self.height and self.free[y,x]):
                return False
        points = np.linspace(start,end,max(2,math.ceil(math.dist(start,end)/self.resolution*4)+1))-self.origin
        c,s = math.cos(self.yaw),math.sin(self.yaw)
        xs = np.floor((c*points[:,0]+s*points[:,1])/self.resolution).astype(np.int64)
        ys = np.floor((-s*points[:,0]+c*points[:,1])/self.resolution).astype(np.int64)
        if not np.all((xs>=0)&(xs<self.width)&(ys>=0)&(ys<self.height)) or not np.all(self.free[ys,xs]):
            return False
        diagonal = (xs[1:]!=xs[:-1])&(ys[1:]!=ys[:-1])
        return bool(np.all(self.free[ys[:-1][diagonal],xs[1:][diagonal]]) and
                    np.all(self.free[ys[1:][diagonal],xs[:-1][diagonal]]))

    def route(self,start,goal,max_distance=80.):
        """Few visible route checkpoints around observed corners, excluding start."""
        distances = self.reachable(start,max_distance)
        x,y = self.cell(goal)
        if not (0<=x<self.width and 0<=y<self.height and distances[y,x]>=0):
            return []
        reverse = [goal,self.world(x,y)]
        while distances[y,x]>0:
            choices = [(nx,ny) for nx,ny in ((x+1,y),(x-1,y),(x,y+1),(x,y-1))
                       if 0<=nx<self.width and 0<=ny<self.height and distances[ny,nx]==distances[y,x]-1]
            x,y = min(choices,key=lambda p:math.dist(self.world(*p),start))
            reverse.append(self.world(x,y))
        path = [start]+list(reversed(reverse))
        route,index = [],0
        while index<len(path)-1:
            next_index = next((i for i in range(len(path)-1,index,-1) if self.segment_free(path[index],path[i])),None)
            if next_index is None:
                return []
            route.append(path[next_index])
            index = next_index
        return route

    def route_towards(self,start,goal,visited=()):
        route = self.route(start,goal)
        if route:
            return route,True
        # A visually detected table may be beyond the mapped floor. Advance
        # through known-free space to reveal its approach, without declaring
        # arrival or routing through the unknown cells around that table.
        distances = self.reachable(start,80.)
        ys,xs = np.nonzero(distances>0)
        if not len(xs):
            return [],False
        gx,gy = self.cell(goal)
        remaining = np.hypot(xs-gx,ys-gy)*self.resolution
        valid = remaining<math.dist(start,goal)-.4
        for point in visited:
            px,py = self.cell(point)
            valid &= np.hypot(xs-px,ys-py)*self.resolution>.6
        scores = np.where(valid,remaining+.08*distances[ys,xs]*self.resolution,np.inf)
        index = int(np.argmin(scores))
        if not math.isfinite(scores[index]):
            return [],False
        return self.route(start,self.world(int(xs[index]),int(ys[index]))),False

    def information_batch(self,cells,view_range=3.):
        if not cells:
            return []
        angles = np.linspace(-math.pi,math.pi,32,endpoint=False)
        steps = np.arange(1,min(max(self.width,self.height),math.ceil(view_range/self.resolution))+1)
        offsets = np.stack((np.cos(angles)[:,None]*steps,np.sin(angles)[:,None]*steps),axis=-1)
        rays = np.rint(np.asarray(cells)[:,None,None,:]+offsets[None,:,:,:]).astype(np.int32)
        x,y = rays[:,:,:,0],rays[:,:,:,1]
        inside = (x>=0)&(y>=0)&(x<self.width)&(y<self.height)
        values = self.data[np.clip(y,0,self.height-1),np.clip(x,0,self.width-1)]
        hit = ~inside|(values<0)|(values>=20)
        first = np.argmax(hit,axis=2)[:,:,None]
        unknown = np.take_along_axis(values,first,axis=2)[:,:,0]<0
        valid = unknown & np.any(hit,axis=2) & np.take_along_axis(inside,first,axis=2)[:,:,0]
        indices = np.take_along_axis(y*self.width+x,first,axis=2)[:,:,0]
        return [int(len(np.unique(row[mask]))) for row,mask in zip(indices,valid)]

    def viewpoints(self,robot,visited=(),failed=(),people=(),max_distance=80.,limit=120,heading=None):
        """People are sensed map-frame (x,y,vx,vy,radius) tuples at current time."""
        distances = self.reachable(robot,max_distance)
        stride = max(1,round(.8/self.resolution))
        cells = [(int(distances[y,x]),x,y) for y in range(0,self.height,stride)
                 for x in range(0,self.width,stride) if distances[y,x]>0]
        candidates = []
        for steps,x,y in sorted(cells):
            point = self.world(x,y)
            distance = math.dist(point,robot)
            if distance<.6 or any(math.dist(point,p[:2])<.9 for p in failed):
                continue
            yaw = math.atan2(point[1]-robot[1],point[0]-robot[0])
            novelty = view_novelty(point,yaw,visited)
            if novelty<1.1:
                continue
            risk = 0.
            for px,py,vx,vy,radius in people:
                clearance = closest_person_distance((px,py),(vx,vy),point,0.)-radius
                if clearance<self.radius+.4:
                    risk = math.inf
                    break
                risk += 1./max(.2,clearance)
            if not math.isfinite(risk):
                continue
            candidates.append((steps,x,y,point,novelty,risk))
        # Cover the reachable distance range instead of spending the whole
        # candidate budget on nearby floor in a large atrium.
        if len(candidates)>limit:
            candidates = [candidates[i] for i in np.linspace(0,len(candidates)-1,limit,dtype=int)]
        result = []
        gains = self.information_batch([(x,y) for _,x,y,_,_,_ in candidates])
        for (steps,x,y,point,novelty,risk),gain in zip(candidates,gains):
            cost = steps*self.resolution
            yaw = math.atan2(point[1]-robot[1],point[0]-robot[0])
            turn = abs(math.atan2(math.sin(yaw-heading),math.cos(yaw-heading))) if heading is not None else 0.
            score = view_score(cost,gain,novelty,turn,risk,self.turn_rate)
            result.append(SearchView(*point,yaw,
                                      score,cost,gain,'frontier' if gain else 'visual_coverage'))
        if heading is not None and visited:
            x,y = self.cell(robot)
            if 0<=x<self.width and 0<=y<self.height and distances[y,x]>=0:
                for offset in (-math.pi/3,math.pi/3):
                    yaw = heading+offset
                    novelty = view_novelty(robot,yaw,visited)
                    if novelty>=1.1:
                        result.append(SearchView(*robot,yaw,view_score(0.,0,novelty,abs(offset),0.,self.turn_rate),
                                                 0.,0,'heading_inspection'))
        return sorted(result,key=lambda goal:(-goal.score,goal.path_length,goal.x,goal.y))

    def support_viewpoints(self,robot,surfaces,visited=(),people=(),heading=0.):
        """Task-independent table views from observed RGB-D support geometry."""
        reachable = self.reachable(robot,80.)
        result = []
        for surface in surfaces:
            target = surface[:2]
            explained = len(surface)>2 and surface[2]<1.
            for offset in np.linspace(0.,2*math.pi,12,endpoint=False):
                point = (target[0]+1.5*math.cos(offset),target[1]+1.5*math.sin(offset))
                x,y = self.cell(point)
                if not (0<=x<self.width and 0<=y<self.height and reachable[y,x]>=0):
                    continue
                yaw = math.atan2(target[1]-point[1],target[0]-point[0])
                novelty = view_novelty(point,yaw,visited)
                if novelty<1.1 or any(closest_person_distance((px,py),(vx,vy),point,0.)<radius+self.radius+.4
                                     for px,py,vx,vy,radius in people):
                    continue
                # Floor visibility cannot see through the table's occupied cell.
                # Check the ray up to the near table edge instead of its centre.
                near = (target[0]+.55*math.cos(offset),target[1]+.55*math.sin(offset))
                if not self.visible_points(point,[near])[0]:
                    continue
                cost = float(reachable[y,x]*self.resolution)
                bearing = math.atan2(point[1]-robot[1],point[0]-robot[0])
                turn = (abs(math.atan2(math.sin(bearing-heading),math.cos(bearing-heading)))+
                        abs(math.atan2(math.sin(yaw-bearing),math.cos(yaw-bearing))))
                gain = 3 if explained else 16
                kind = 'known_support_inspection' if explained else 'support_inspection'
                result.append(SearchView(*point,yaw,view_score(cost,gain,novelty,turn,0.,self.turn_rate),cost,0,kind))
        return sorted(result,key=lambda goal:-goal.score)

    def semantic_viewpoints(self,robot,target,people=()):
        """Reobserve historical sensor geometry from a reachable stand-off ring."""
        distances = self.reachable(robot,80.)
        result = []
        bearing = math.atan2(robot[1]-target[1],robot[0]-target[0])
        for offset in np.linspace(0.,2*math.pi,16,endpoint=False):
            point = (target[0]+1.4*math.cos(bearing+offset),target[1]+1.4*math.sin(bearing+offset))
            x,y = self.cell(point)
            if not (0<=x<self.width and 0<=y<self.height and distances[y,x]>=0):
                continue
            if any(closest_person_distance((px,py),(vx,vy),point,0.)<radius+self.radius+.4 for px,py,vx,vy,radius in people):
                continue
            cost = float(distances[y,x]*self.resolution)
            result.append(SearchView(*point,math.atan2(target[1]-point[1],target[0]-point[0]),100.-cost,cost,0,'semantic_reobserve'))
        return sorted(result,key=lambda goal:goal.path_length)

    def clearance_recovery_view(self,robot,heading,physical_radius,people=()):
        """Leave an inflated search reserve through a checked physical footprint.

        Only straight body-axis moves are offered, matching precision control.
        Unknown space and actual body clearance are never waived.
        """
        if self.segment_free(robot,robot):
            return None
        physical = ObservedSearchMap(self.message,physical_radius)
        if not physical.segment_free(robot,robot):
            return None
        for distance in np.arange(.2,.91,.1):
            for offset in (0.,math.pi/2,-math.pi/2,math.pi):
                direction = heading+offset
                goal = robot[0]+distance*math.cos(direction),robot[1]+distance*math.sin(direction)
                if not self.segment_free(goal,goal) or not physical.segment_free(robot,goal):
                    continue
                if any(closest_person_distance((px,py),(vx,vy),goal,0.)<radius+self.radius+.4
                       for px,py,vx,vy,radius in people):
                    continue
                return goal
        return None
