"""Bounded planning reserve for recently observed velocity-estimate changes."""
from collections import deque
import math

import numpy as np


class ForecastChangeReserve:
    """Use past sensed updates, never future truth or a smoothed-away new state.

    The reserve is an engineering guard, not a calibrated confidence interval.
    """
    def __init__(self,window=.5,deadband=.03,cap=.35):
        self.window,self.deadband,self.cap=window,deadband,cap
        self.previous={}
        self.changes={}

    def observe(self,stamp,tracks):
        identities=set()
        for track in tracks:
            identities.add(track.track_id)
            if not all(math.isfinite(v) for v in (stamp,track.vx,track.vy)):
                continue
            previous=self.previous.get(track.track_id)
            if previous is not None and stamp<=previous[0]:
                continue
            if previous is not None:
                delta=max(0.,math.hypot(track.vx-previous[1],track.vy-previous[2])-self.deadband)
                self.changes.setdefault(track.track_id,deque(maxlen=32)).append((stamp,delta))
            self.previous[track.track_id]=(stamp,track.vx,track.vy)
        self.previous={key:value for key,value in self.previous.items() if key in identities}
        self.changes={key:value for key,value in self.changes.items() if key in identities}

    def reserve(self,tracks,times,now,hold_s=2.):
        changes=[max((delta for stamp,delta in self.changes.get(t.track_id,())
                      if 0.<=now-stamp<=self.window),default=0.) for t in tracks]
        return np.minimum(self.cap,(np.asarray(times)[:,None]+hold_s)*np.asarray(changes)[None,:])
