"""Bounded retry of absolute motion goals after localization interruptions."""
from functools import wraps
from inspect import signature
import time


class LocalizationInterrupted(RuntimeError):
    """Fresh measured odometry is unavailable; motion must terminate first."""


class BodyInstabilityError(RuntimeError):
    """Sustained measured body tilt; not a retryable localization outage."""


class ViewpointDisplaced(RuntimeError):
    """Inspection drift guard completed braking; reobserve/replan, not blind resume.

    Raised only after measured settling; the motion owner's finally block revokes
    intent before a caller handles it. It is distinct from failed braking or falls.
    """


def recover_localization(operation):
    parameters = signature(operation)
    @wraps(operation)
    def execute(self,*args,**kwargs):
        bound = parameters.bind(self,*args,**kwargs)
        bound.apply_defaults()
        deadline = time.monotonic()+bound.arguments['timeout']
        for attempt in range(3):
            remaining = deadline-time.monotonic()
            if remaining<=0.:
                raise RuntimeError('motion deadline exceeded during localization recovery')
            bound.arguments['timeout'] = remaining
            try:
                return operation(*bound.args,**bound.kwargs)
            except LocalizationInterrupted:
                # The operation's action context/finally has already run. An
                # unconfirmed action termination raises its own error and is
                # deliberately not retried here.
                self.stop_intent()
                if attempt==2:
                    raise
                self.wait_localization(deadline,attempt+1)
        raise AssertionError('unreachable')
    return execute
