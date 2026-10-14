"""Bounded, conservative static-clearance queries without SciPy dependencies."""

import math
import numpy as np


def observed_free_mask(points, pose, ranges, angle_min, angle_increment):
    """Clear remembered returns only when three measured rays see beyond them."""
    points = np.asarray(points)
    if not len(points) or len(ranges) < 3:
        return np.zeros(len(points), dtype=bool)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    local = (points - pose[:2]) @ np.asarray(((c, -s), (s, c)))
    distance = np.linalg.norm(local, axis=1)
    indexes = np.rint((np.arctan2(local[:, 1], local[:, 0]) - angle_min) / angle_increment).astype(int)
    valid = (indexes > 0) & (indexes < len(ranges) - 1)
    bins = np.clip(indexes[:, None] + (-1, 0, 1), 0, len(ranges) - 1)
    measured = np.asarray(ranges)[bins]
    return valid & np.all(np.isfinite(measured) & (measured > distance[:, None] + 0.15), axis=1)


class StaticDistanceField:
    def __init__(self, points, origin, extent, influence, resolution=0.10):
        self.points = np.asarray(points, dtype=float).reshape((-1,2))
        self.resolution = resolution
        self.origin = np.asarray(origin, dtype=float) - extent
        self.width = math.ceil(2.0 * extent / resolution) + 1
        steps = math.ceil(influence / resolution) + 2
        field = np.full((self.width, self.width), steps + 1, dtype=np.int16)
        cells = np.floor((np.asarray(points) - self.origin) / resolution).astype(int)
        valid = np.all((cells >= 0) & (cells < self.width), axis=1)
        cells = cells[valid]
        field[cells[:, 1], cells[:, 0]] = 0
        for _ in range(steps):
            padded = np.pad(field, 1, constant_values=steps + 1)
            nearby = np.minimum.reduce([padded[y:y + self.width, x:x + self.width]
                                       for y in range(3) for x in range(3)])
            field = np.minimum(field, nearby + 1)
        # Chebyshev distance is a lower bound on Euclidean distance; subtract
        # one cell for unknown sub-cell locations. Never overstate clearance.
        self.distances = np.maximum(0.0, (field.astype(float) - 1.0) * resolution)

    def query(self, points):
        cells = np.floor((np.asarray(points) - self.origin) / self.resolution).astype(int)
        valid = np.all((cells >= 0) & (cells < self.width), axis=1)
        result = np.zeros(len(cells))  # Outside the checked window is not free.
        result[valid] = self.distances[cells[valid, 1], cells[valid, 0]]
        return result

    def trajectory_is_blocked(self, positions, clearance, lower_bounds=None, recovery_radius=None):
        """Refine ambiguous grid bounds with exact point/segment distances.

        A conservative grid is a cheap proof of clearance, not a proof of
        collision. Quantization must not trap the robot beside a clear wall.
        Swept segments also catch obstacles between trajectory samples.
        """
        positions = np.asarray(positions, dtype=float)
        if not len(positions) or not np.all(np.isfinite(positions)):
            return True
        cells = np.floor((positions-self.origin)/self.resolution).astype(int)
        if not np.all((cells >= 0) & (cells < self.width)):
            return True
        lower_bounds = self.query(positions) if lower_bounds is None else lower_bounds
        delta = np.diff(positions, axis=0)
        half_step = .5*float(np.max(np.linalg.norm(delta,axis=1))) if len(delta) else 0.
        if np.all(lower_bounds >= clearance+half_step):
            return False
        if not len(self.points):
            return False
        if not len(delta):
            return bool(np.any(np.sum((self.points-positions[0])**2,axis=1) < clearance**2))
        relative = self.points[None,:,:]-positions[:-1,None,:]
        fraction = np.clip(np.einsum('ijk,ik->ij',relative,delta) /
                           np.maximum(1e-12,np.sum(delta*delta,axis=1))[:,None],0.,1.)
        separation = relative-fraction[:,:,None]*delta[:,None,:]
        minimum_squared = float(np.min(np.sum(separation*separation,axis=2)))
        if minimum_squared >= clearance**2:
            return False
        if recovery_radius is not None:
            vertex_clearance = np.min(np.linalg.norm(positions[:,None,:]-self.points[None,:,:],axis=2),axis=1)
            initial = float(vertex_clearance[0])
            if (recovery_radius <= initial < clearance and minimum_squared >= initial**2-1e-9 and
                    np.all(np.diff(vertex_clearance) >= -1e-6) and vertex_clearance[-1] >= clearance+.01):
                # Only leave an already violated reserve. Never enter a new
                # violation, reduce existing clearance, or cross body geometry.
                return False
        return True

    def first_violation_time(self,positions,times,clearance,recovery_radius=None):
        """Conservative segment-start time of a future clearance violation."""
        positions,times = np.asarray(positions),np.asarray(times)
        cells = np.floor((positions-self.origin)/self.resolution).astype(int)
        outside = np.any((cells < 0) | (cells >= self.width),axis=1)
        first = float(np.min(times[outside])) if np.any(outside) else math.inf
        if not len(self.points) or len(positions)<2:
            return first
        initial = float(np.min(np.linalg.norm(self.points-positions[0],axis=1)))
        if recovery_radius is not None and recovery_radius <= initial < clearance:
            clearance = max(recovery_radius,initial-1e-6)
        delta = np.diff(positions,axis=0)
        relative = self.points[None,:,:]-positions[:-1,None,:]
        fraction = np.clip(np.einsum('ijk,ik->ij',relative,delta) /
                           np.maximum(1e-12,np.sum(delta*delta,axis=1))[:,None],0.,1.)
        separation = np.linalg.norm(relative-fraction[:,:,None]*delta[:,None,:],axis=2)
        hits = np.any(separation < clearance,axis=1)
        return min(first,float(np.min(times[:-1][hits]))) if np.any(hits) else first


class StaticFieldCache:
    """One immutable geometry snapshot, invalidated by actual obstacle contents.

    Quantized origin reuse adds window padding, never reuses clearance after an
    obstacle insertion/removal, and keeps out-of-window queries conservative.
    """
    def __init__(self):
        self.key = self.field = None

    def get(self,points,origin,extent,influence):
        points = np.asarray(points,dtype=float).reshape(-1,2)
        anchor = tuple(round(float(value)/.25)*.25 for value in origin)
        key = (points.shape,points.tobytes(),anchor,extent,influence)
        if key!=self.key:
            self.field = StaticDistanceField(points.copy(),anchor,extent+.25,influence)
            self.key = key
        return self.field
