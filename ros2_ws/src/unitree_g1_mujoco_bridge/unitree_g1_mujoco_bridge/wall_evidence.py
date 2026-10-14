"""Sensor-only evidence that a narrow cluster belongs to a taller wall sheet."""
import numpy as np


class WallEvidence:
    def __init__(self,points,upper_height,structural_points=()):
        cloud = np.asarray(points,dtype=float).reshape(-1,3)
        self.upper = cloud[np.isfinite(cloud).all(axis=1)&(cloud[:,2]>upper_height)]
        self.cache = {}
        structure = np.asarray(structural_points,dtype=float).reshape(-1,3)
        self.structure = structure[np.isfinite(structure).all(axis=1)&(structure[:,2]>.15)]

    def supports(self,x,y,body_sheet=True):
        key = float(x),float(y),body_sheet
        if key not in self.cache:
            self.cache[key] = ('wall_continuation' if self._supports(x,y) else
                               'body_height_wall_sheet' if body_sheet and self._body_height_sheet(x,y) else '')
        self.last_support_kind = self.cache[key]
        return bool(self.cache[key])

    def _supports(self,x,y):
        # Only a locally extended, thin sheet above the admitted person-height
        # range counts. Stationarity alone is deliberately not evidence of wall.
        centre = np.array((x,y))
        delta = self.upper[:,:2]-centre
        nearby = self.upper[np.sum(delta*delta,axis=1)<=1.5**2]
        if len(nearby)>600:
            nearby = nearby[np.linspace(0,len(nearby)-1,600,dtype=int)]
        if len(nearby)<6:
            return False
        # Bounded deterministic line proposals tolerate two walls at a corner.
        sample = nearby[np.linspace(0,len(nearby)-1,min(24,len(nearby)),dtype=int),:2]
        a,b = np.triu_indices(len(sample),1)
        vectors = sample[b]-sample[a]
        lengths = np.linalg.norm(vectors,axis=1)
        valid = np.flatnonzero(lengths>.4)
        if not len(valid):
            return False
        for index in valid[np.linspace(0,len(valid)-1,min(32,len(valid)),dtype=int)]:
            direction = vectors[index]/lengths[index]
            normal = np.array((-direction[1],direction[0]))
            origin = sample[a[index]]
            if abs(float((centre-origin)@normal))>.10:
                continue
            inliers = nearby[np.abs((nearby[:,:2]-origin)@normal)<.06]
            if len(inliers)<6:
                continue
            along = (inliers[:,:2]-origin)@direction
            # An overhead beam alone is not a vertical wall. Require both
            # horizontal extent and substantial vertical continuation.
            if (np.quantile(along,.95)-np.quantile(along,.05)>=.55 and
                    np.quantile(inliers[:,2],.9)-np.quantile(inliers[:,2],.1)>=.4):
                return True
        return False

    def _body_height_sheet(self,x,y):
        """Near walls can lie outside the sensor's upper-return field of view.

        Require a thin extended vertical sheet across three horizontal columns,
        each spanning substantial height. A flat torso, arms, tabletop, or a
        stationary person's centroid alone cannot supply this evidence.
        """
        centre = np.array((x,y))
        delta = self.structure[:,:2]-centre
        nearby = self.structure[np.sum(delta*delta,axis=1)<=1.2**2]
        if len(nearby)<24:
            return False
        if len(nearby)>600:
            nearby = nearby[np.linspace(0,len(nearby)-1,600,dtype=int)]
        central = np.linalg.norm(nearby[:,:2]-centre,axis=1)<.35
        if np.count_nonzero(central)<8:
            return False
        sample = nearby[np.linspace(0,len(nearby)-1,min(24,len(nearby)),dtype=int),:2]
        a,b = np.triu_indices(len(sample),1)
        vectors = sample[b]-sample[a]
        lengths = np.linalg.norm(vectors,axis=1)
        valid = np.flatnonzero(lengths>.5)
        if not len(valid):
            return False
        for index in valid[np.linspace(0,len(valid)-1,min(32,len(valid)),dtype=int)]:
            direction = vectors[index]/lengths[index]
            normal = np.array((-direction[1],direction[0]))
            origin = sample[a[index]]
            if abs(float((centre-origin)@normal))>.08:
                continue
            inliers = np.abs((nearby[:,:2]-origin)@normal)<.025
            if np.count_nonzero(inliers)<24 or np.mean(inliers[central])<.7:
                continue
            points = nearby[inliers]
            along = (points[:,:2]-origin)@direction
            lo,hi = np.quantile(along,(.05,.95))
            if hi-lo<.65 or np.ptp(points[:,2])<1.25 or np.max(points[:,2])<1.35:
                continue
            span = hi-lo
            columns = [points[np.abs(along-centre)<.1*span,2]
                       for centre in (lo+.1*span,(lo+hi)/2,hi-.1*span)]
            if all(len(column)>=4 and np.ptp(column)>=.9 for column in columns):
                return True
        return False
