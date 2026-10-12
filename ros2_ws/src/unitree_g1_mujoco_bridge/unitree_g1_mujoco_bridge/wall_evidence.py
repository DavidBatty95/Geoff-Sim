"""Sensor-only evidence that a narrow cluster belongs to a taller wall sheet."""
import numpy as np


class WallEvidence:
    def __init__(self,points,upper_height):
        cloud = np.asarray(points,dtype=float).reshape(-1,3)
        self.upper = cloud[np.isfinite(cloud).all(axis=1)&(cloud[:,2]>upper_height)]

    def supports(self,x,y):
        # Only a locally extended, thin sheet above the admitted person-height
        # range counts. Stationarity alone is deliberately not evidence of wall.
        centre = np.array((x,y))
        delta = self.upper[:,:2]-centre
        nearby = self.upper[np.sum(delta*delta,axis=1)<=1.5**2]
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
