"""Bounded planar scan registration. No simulator, map, or command inputs."""
import math

import numpy as np


def rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((c, -s), (s, c)))


def voxel_points(points, size=0.12):
    points = np.asarray(points, dtype=float)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        return points
    _, indices = np.unique(np.floor(points / size).astype(np.int64), axis=0, return_index=True)
    return points[indices]


def registration_points(points):
    """Prefer measured upper structure over moving bodies when observable."""
    points = np.asarray(points)
    radius = np.linalg.norm(points[:,:2],axis=1)
    visible = (radius > 1.) & (radius < 18.)
    upper = points[visible & (points[:,2] > 2.) & (points[:,2] < 3.2),:2]
    if len(voxel_points(upper)) >= 60:
        return upper,'upper_structure'
    return points[visible & (points[:,2] > .25) & (points[:,2] < 1.8),:2],'full_band'


class ScanOdometry:
    """Trimmed scan-to-keyframe ICP with measured IMU heading increments.

    Spatial hashing bounds correspondence work rather than allocating N x M
    distance matrices. Rejected registrations never update the reference/pose.
    """
    def __init__(self):
        self.reference = None
        self.xy = np.zeros(2)
        self.yaw = 0.0
        self.initial_heading = None
        self.reference_xy = np.zeros(2)
        self.reference_yaw = 0.
        self.cell_size = 0.20
        self.offsets = np.array([(x, y) for x in range(-2, 3) for y in range(-2, 3)])
        self.quality = {}
        self.fallback = None

    @staticmethod
    def _keys(cells):
        return cells[..., 0] * 1000003 + cells[..., 1]

    def _set_reference(self, points):
        cells = np.floor(points / self.cell_size).astype(np.int64)
        keys, indices = np.unique(self._keys(cells), return_index=True)
        self.keys, self.reference = keys, points[indices]
        self.reference_xy = self.xy.copy()
        self.reference_yaw = self.yaw
        neighbours = self._keys(cells[indices,None,:]+self.offsets)
        lookup = np.minimum(np.searchsorted(self.keys,neighbours),len(self.keys)-1)
        valid = self.keys[lookup] == neighbours
        count = np.maximum(1,np.sum(valid,axis=1))
        coordinates = self.reference[lookup]
        mean = np.sum(coordinates*valid[:,:,None],axis=1)/count[:,None]
        centred = coordinates-mean[:,None,:]
        covariance = np.einsum('nki,nkj,nk->nij',centred,centred,valid)/count[:,None,None]
        eigenvalues,eigenvectors = np.linalg.eigh(covariance)
        self.normals = eigenvectors[:,:,0]
        self.normal_valid = (count >= 3) & (eigenvalues[:,1] > 1e-5) & (eigenvalues[:,0] < .1*eigenvalues[:,1])

    def update(self,points,heading,fallback_points=None):
        """Prefer upper structure but retain a pose-anchored full-band reference.

        Sparse upper returns can disappear at a doorway or near a wall. A
        fallback must independently pass the same registration gates; failure
        of both bands never creates a fresh pose.
        """
        accepted = self._update(points,heading)
        if not accepted and fallback_points is not None and self.fallback is not None:
            rejection = dict(self.quality)
            self.fallback.xy,self.fallback.yaw = self.xy.copy(),self.yaw
            if self.fallback._update(fallback_points,heading):
                self.xy,self.yaw = self.fallback.xy.copy(),self.fallback.yaw
                self.quality = dict(self.fallback.quality,registration_fallback=True,primary_rejection=rejection)
                primary = voxel_points(points)
                if len(primary)>=60:
                    self._set_reference(primary@rotation(self.yaw).T+self.xy)
                accepted = True
        if accepted and fallback_points is not None:
            full = voxel_points(fallback_points)
            if len(full)>=60:
                if self.fallback is None:
                    self.fallback = ScanOdometry()
                self.fallback.initial_heading = self.initial_heading
                self.fallback.xy,self.fallback.yaw = self.xy.copy(),self.yaw
                self.fallback._set_reference(full@rotation(self.yaw).T+self.xy)
        return accepted

    def _update(self, points, heading):
        points = voxel_points(points)
        if len(points) < 60 or not math.isfinite(heading):
            self.quality = {"accepted": False, "reason": "insufficient_returns"}
            return False
        if self.initial_heading is None:
            self.initial_heading = heading
        yaw = math.atan2(math.sin(heading - self.initial_heading), math.cos(heading - self.initial_heading))
        rotated = points @ rotation(yaw).T
        if self.reference is None:
            self._set_reference(rotated)
            self.quality = {"accepted": True, "inliers": len(points), "rmse": 0.0}
            return True
        estimate = self.xy.copy()
        method = 'point_to_point'
        for _ in range(8):
            transformed = rotated + estimate
            cells = np.floor(transformed / self.cell_size).astype(np.int64)
            keys = self._keys(cells[:, None, :] + self.offsets)
            indices = np.searchsorted(self.keys, keys)
            indices = np.minimum(indices, len(self.keys) - 1)
            valid = self.keys[indices] == keys
            residual = self.reference[indices] - transformed[:, None, :]
            distances = np.sum(residual ** 2, axis=2)
            distances[~valid] = np.inf
            nearest = np.argmin(distances, axis=1)
            best = distances[np.arange(len(points)), nearest]
            good = best < 0.35 ** 2
            if good.sum() < 50:
                self.quality = {"accepted": False, "reason": "poor_overlap"}
                return False
            # Moving bodies and changing occlusion edges must not pull ego motion.
            good &= best <= max(0.025 ** 2, np.quantile(best[good], 0.75))
            corrections = residual[np.arange(len(points)), nearest][good]
            delta = np.mean(corrections, axis=0)
            matches = indices[np.arange(len(points)),nearest]
            plane_mask = good & self.normal_valid[matches]
            if np.count_nonzero(plane_mask) >= 30:
                normals = self.normals[matches[plane_mask]]
                plane_residual = residual[np.arange(len(points)),nearest][plane_mask]
                hessian = normals.T@normals
                spectrum = np.linalg.eigvalsh(hessian)
                if spectrum[0] > .02*spectrum[-1]:
                    delta = np.linalg.solve(hessian,normals.T@np.sum(normals*plane_residual,axis=1))
                    method = 'point_to_plane'
            estimate += delta
            if np.linalg.norm(delta) < 0.001:
                break
        rmse = float(np.sqrt(np.mean(best[good])))
        accepted = rmse < 0.18 and np.linalg.norm(estimate - self.xy) < 0.30
        self.quality = {"accepted": bool(accepted), "inliers": int(good.sum()), "rmse": rmse, "method": method}
        if not accepted:
            return False
        self.xy, self.yaw = estimate, yaw
        if (np.linalg.norm(self.xy - self.reference_xy) > 0.45 or
                abs(math.atan2(math.sin(yaw-self.reference_yaw),math.cos(yaw-self.reference_yaw)))>.45):
            self._set_reference(rotated + self.xy)
        return True
