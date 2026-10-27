"""Pure escape search plus a single-flight worker. No worker accesses a ROS node."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import math
import time

from .bridge_safety import trajectory_static_cost
from .motion_envelope import MotionEnvelope
from .prediction_risk import PreparedPredictions,time_aligned_prediction_risk


@dataclass(frozen=True)
class EscapeSnapshot:
    pose: tuple
    seconds: float
    horizon: float
    full_horizon: float
    envelope: MotionEnvelope
    velocity: tuple
    filtered: tuple | None
    predictions: PreparedPredictions
    prediction_epoch: float
    obstacles: tuple
    static_clearance: float
    influence_range: float
    response_delay: float
    uncertainty_scale: float
    minimum_confidence: float
    recovery_clearance: float
    stop_clearance: float
    preferred: tuple


def rollout(snapshot,command,horizon=None):
    return snapshot.envelope.rollout(snapshot.pose,command,snapshot.velocity,
        snapshot.horizon if horizon is None else horizon,.1,snapshot.response_delay,
        filtered_command=snapshot.filtered)


def risk(snapshot,command,trajectory,horizon=None,minimum_time=0.):
    return time_aligned_prediction_risk(snapshot.predictions,snapshot.seconds-snapshot.prediction_epoch,
        snapshot.pose,command,snapshot.horizon if horizon is None else horizon,.1,math.inf,
        snapshot.uncertainty_scale,snapshot.minimum_confidence,trajectory,minimum_time=minimum_time)


def stopped_tail(snapshot):
    value = risk(snapshot,(0.,0.,0.),rollout(snapshot,(0.,0.,0.)),minimum_time=max(0.,snapshot.horizon-.3))
    return None if value is None else value[0]


def checked_escape(snapshot,proposed,stop_tail):
    """Current-state short-horizon admission predicate, shared by search/recheck."""
    command = snapshot.envelope.normalize(proposed)
    if not any(command):
        return None
    trajectory = rollout(snapshot,command)
    x0,y0,yaw0 = snapshot.pose
    c,s = math.cos(yaw0),math.sin(yaw0)
    local = [(c*(x-x0)+s*(y-y0),-s*(x-x0)+c*(y-y0),yaw-yaw0) for x,y,yaw,_ in trajectory]
    if trajectory_static_cost(local,snapshot.obstacles,snapshot.static_clearance,snapshot.influence_range)[1]:
        return None
    conflict = risk(snapshot,command,trajectory)
    if conflict is None or conflict[0]<snapshot.recovery_clearance or conflict[0]<snapshot.stop_clearance-.001:
        return None
    tail_risk = risk(snapshot,command,trajectory,minimum_time=max(0.,snapshot.horizon-.3))
    tail = None if tail_risk is None else tail_risk[0]
    if conflict[0]<=snapshot.stop_clearance+.05 and (tail is None or stop_tail is None or tail<=stop_tail+.05):
        return None
    return command,conflict,tail


def search_escape(snapshot):
    """Propose a velocity; the supervisor must revalidate it before publication."""
    stop_tail = stopped_tail(snapshot)
    envelope = snapshot.envelope
    commands = [snapshot.preferred]+[(vx,vy,0.)
        for vx in (-min(.45,envelope.max_x),0.,.5*envelope.max_x,envelope.max_x)
        for vy in (-envelope.max_y,0.,envelope.max_y)]
    best = best_rank = None
    for proposed in commands:
        checked = checked_escape(snapshot,proposed,stop_tail)
        if checked is None:
            continue
        command,conflict,tail = checked
        forecast = risk(snapshot,command,rollout(snapshot,command,snapshot.full_horizon),snapshot.full_horizon)
        rank = ((forecast or conflict)[0],conflict[0],tail if tail is not None else -math.inf)
        if best_rank is None or rank>best_rank:
            best,best_rank = command,rank
    return best


@dataclass(frozen=True)
class SearchResult:
    generation: int
    submitted_at: float
    completed_at: float
    snapshot: EscapeSnapshot
    command: tuple | None


def execute_search(search,snapshot,generation,submitted,clock):
    """Top-level entry point also supports a process-isolated search executor."""
    command=search(snapshot)
    return SearchResult(generation,submitted,clock(),snapshot,command)


class EscapeSearchWorker:
    """One owning control thread; no queued stale snapshots and no blocking poll."""
    def __init__(self,search=search_escape,clock=time.monotonic,executor=None):
        self.search,self.clock = search,clock
        self.executor = executor or ThreadPoolExecutor(max_workers=1,thread_name_prefix='collision-escape')
        self.future = None
        self.generation = 0
        self.closed = False
        self.counters = dict(submitted=0,completed=0,failed=0,discarded=0,expired=0,busy=0,proposed=0,no_candidate=0)
        self.last_error = None
        self.last_search_ms = None

    def submit(self,snapshot):
        if self.closed:
            return False
        if self.future is not None:
            self.counters['busy'] += 1
            return False
        generation,submitted = self.generation,self.clock()
        self.future = self.executor.submit(execute_search,self.search,snapshot,generation,submitted,self.clock)
        self.counters['submitted'] += 1
        return True

    def poll(self,max_age):
        if self.future is None or not self.future.done():
            return None
        future,self.future = self.future,None
        try:
            result = future.result()
        except Exception as exc:
            self.counters['failed'] += 1
            self.last_error = str(exc)
            return None
        self.counters['completed'] += 1
        self.last_search_ms = (result.completed_at-result.submitted_at)*1000.
        if result.generation != self.generation:
            self.counters['discarded'] += 1
            return None
        age = self.clock()-result.submitted_at
        if not math.isfinite(age) or not 0.<=age<=max_age:
            self.counters['expired'] += 1
            return None
        self.counters['no_candidate' if result.command is None else 'proposed'] += 1
        return result

    def invalidate(self):
        self.generation += 1

    def diagnostics(self):
        return dict(self.counters,generation=self.generation,pending=self.future is not None,
                    running=self.future is not None and not self.future.done(),
                    last_error=self.last_error,last_search_ms=self.last_search_ms)

    def shutdown(self):
        self.closed = True
        self.invalidate()
        self.executor.shutdown(wait=True,cancel_futures=True)
