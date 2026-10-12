"""Coarse observed-map regions and coverage decisions, without layout inputs.

Regions approximate rooms/corridors. Coverage is a 2-D visibility proxy, never
proof of object absence. Geometry is rebuilt as the observed map grows; memory
uses map-frame points rather than assuming region IDs remain stable.
"""
from collections import deque
import math

import numpy as np

from .exploration_search import SearchView


class RoomTopology:
    def __init__(self,grid):
        self.grid = grid
        self.stride = max(1,math.ceil(.4/grid.resolution))
        step = self.stride
        height,width = math.ceil(grid.height/step),math.ceil(grid.width/step)
        padded = np.zeros((height*step,width*step),dtype=bool)
        padded[:grid.height,:grid.width] = grid.free
        self.free = padded.reshape(height,step,width,step).all(axis=(1,3))
        # Erosion closes door-sized necks; expanding connected cores partitions
        # the traversable area without connecting through a wall or unknown.
        cells = max(1,math.ceil((1.55-grid.radius)/(step*grid.resolution)))
        size = 2*cells+1
        integral = np.pad(np.pad(self.free.astype(np.int32),cells),((1,0),(1,0))).cumsum(0).cumsum(1)
        core = (integral[size:,size:]-integral[:-size,size:]-integral[size:,:-size]+integral[:-size,:-size])==size*size
        labels = np.zeros((height,width),dtype=np.int32)
        def flood(seed,mask,label):
            queue = deque([seed])
            labels[seed[1],seed[0]] = label
            while queue:
                x,y = queue.popleft()
                for nx,ny in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
                    if 0<=nx<width and 0<=ny<height and mask[ny,nx] and labels[ny,nx]==0:
                        labels[ny,nx] = label
                        queue.append((nx,ny))
        count = 0
        for y,x in zip(*np.nonzero(core)):
            if not labels[y,x]:
                count += 1
                flood((int(x),int(y)),core,count)
        cores = {}
        for y,x in zip(*np.nonzero(labels)):
            cores.setdefault(int(labels[y,x]),[]).append((int(x),int(y)))
        labels.fill(0)
        queue = deque()
        count = 0
        for cells_in_core in sorted(cores.values(),key=len,reverse=True)[:128]:
            centre = np.mean(cells_in_core,axis=0)
            x,y = min(cells_in_core,key=lambda point:math.dist(point,centre))
            count += 1
            labels[y,x] = count
            queue.append((x,y,0))
        limit = max(1,round(4./(step*grid.resolution)))
        while queue:
            x,y,distance = queue.popleft()
            if distance>=limit:
                continue
            for nx,ny in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
                if 0<=nx<width and 0<=ny<height and self.free[ny,nx] and labels[ny,nx]==0:
                    labels[ny,nx] = labels[y,x]
                    queue.append((nx,ny,distance+1))
        # Furnished rooms can lack a wide core. Seed remaining space at local
        # clearance maxima, with bounded geodesic extent, rather than letting
        # one atrium core absorb the entire connected building.
        clearance = np.zeros_like(labels)
        eroded = self.free.copy()
        while np.any(eroded):
            clearance += eroded
            padded = np.pad(eroded,1)
            eroded = (padded[1:-1,1:-1]&padded[:-2,1:-1]&padded[2:,1:-1]&
                      padded[1:-1,:-2]&padded[1:-1,2:])
        while count<128 and np.any(self.free&(labels==0)):
            y,x = np.unravel_index(np.argmax(np.where(self.free&(labels==0),clearance,-1)),labels.shape)
            count += 1
            labels[y,x] = count
            queue = deque([(int(x),int(y),0)])
            while queue:
                x,y,distance = queue.popleft()
                if distance>=limit:
                    continue
                for nx,ny in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
                    if 0<=nx<width and 0<=ny<height and self.free[ny,nx] and labels[ny,nx]==0:
                        labels[ny,nx] = count
                        queue.append((nx,ny,distance+1))
        self.labels = labels
        self.points,self.keys = {},{}
        buckets = {}
        for y,x in zip(*np.nonzero(labels)):
            buckets.setdefault(int(labels[y,x]),[]).append(grid.world(int(x)*step+step//2,int(y)*step+step//2))
        for label in sorted(buckets,key=lambda label:len(buckets[label]),reverse=True)[:128]:
            points = np.asarray(buckets[label])
            self.points[label] = points
            self.keys[label] = [self.key(point) for point in points]
        self.labels[~np.isin(self.labels,list(self.points))] = 0

    @staticmethod
    def key(point):
        return round(float(point[0])*2),round(float(point[1])*2)

    def region(self,point):
        x,y = self.grid.cell(point)
        x,y = x//self.stride,y//self.stride
        return int(self.labels[y,x]) if 0<=y<self.labels.shape[0] and 0<=x<self.labels.shape[1] else 0

    def visible(self,position,yaw,current_grid):
        seen = set()
        for points,keys in zip(self.points.values(),self.keys.values()):
            delta = points-np.asarray(position)
            distances = np.linalg.norm(delta,axis=1)
            bearings = np.arctan2(delta[:,1],delta[:,0])-yaw
            valid = (distances>=.5)&(distances<=3.5)&(np.abs(np.arctan2(np.sin(bearings),np.cos(bearings)))<=math.pi/6)
            indices = np.flatnonzero(valid)
            visible = current_grid.visible_points(position,points[indices])
            seen.update(keys[i] for i,clear in zip(indices,visible) if clear)
        return seen

    def decide(self,goals,robot,seen,inspections,anchor=None):
        groups = {}
        for goal in goals:
            region = self.region((goal.x,goal.y))
            if region:
                groups.setdefault(region,[]).append(goal)
        active = self.region(anchor) if anchor is not None else self.region(robot)
        summaries,utilities = [],{}
        for region,points in self.points.items():
            coverage = sum(key in seen for key in self.keys[region])/len(points)
            views = sum(self.region(view[:2])==region for view in inspections)
            candidates = groups.get(region,[])
            gain = max((g.information for g in candidates),default=0)
            deferred = coverage>=.7 or views>=6 or (views>=4 and gain<3)
            summary = dict(region=region,centroid=points.mean(axis=0).tolist(),coverage_proxy=coverage,
                           inspections=views,candidates=len(candidates),deferred=deferred,active=False)
            summaries.append(summary)
            if candidates:
                travel = min(g.path_length for g in candidates)
                utility = (1.+4.*(1.-coverage)+.25*gain)/(1.+.15*travel)
                if region==active and not deferred:
                    utility *= 1.5  # Bounded room commitment reduces doorway ping-pong.
                if deferred:
                    utility *= .08
                utilities[region] = utility
        if not utilities:
            return goals,summaries
        chosen = max(utilities,key=lambda region:(utilities[region],-region))
        for summary in summaries:
            summary['active'] = summary['region']==chosen
        points = self.points[chosen]
        unseen = points[[key not in seen for key in self.keys[chosen]]]
        ranked = []
        for goal in groups[chosen]:
            yaw,best = goal.yaw,0.
            if len(unseen):
                delta = unseen-np.asarray((goal.x,goal.y))
                nearby = np.linalg.norm(delta,axis=1)<=3.5
                indices = np.flatnonzero(nearby)
                nearby[indices] &= self.grid.visible_points((goal.x,goal.y),unseen[indices])
                bearings = np.arctan2(delta[nearby,1],delta[nearby,0])
                # Aim a useful camera view instead of a fixed quarter-turn.
                for offset in ([0.] if goal.kind in ('support_inspection','semantic_reobserve','heading_inspection') else np.linspace(-math.pi/3,math.pi/3,5)):
                    direction = goal.yaw+float(offset)
                    count = int(np.sum(np.abs(np.arctan2(np.sin(bearings-direction),np.cos(bearings-direction)))<=math.pi/6))
                    # Inspection has a physical turning cost. Prefer views
                    # aligned with arrival rather than costly half-turns.
                    value = count/(1.+4.*abs(offset))
                    if value>best:
                        best = value
                        yaw = float(direction)
            score = goal.score+.08*best/(1.+.35*goal.path_length)
            ranked.append(SearchView(goal.x,goal.y,yaw,score,goal.path_length,goal.information,goal.kind))
        return sorted(ranked,key=lambda goal:(-goal.score,goal.path_length)),summaries
