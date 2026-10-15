"""Timing of a specific inference result reaching the joint-target publisher.

This is not an acknowledgement from the bridge, simulator or physical motors.
LowState has no acquisition stamp here; its age is explicitly receipt-based.
"""
import math
import hashlib
import struct


def leg_target_fingerprint(targets):
    """Wire-float target content for legs/waist, excluding composed arm targets.

    This is content identity, not a unique command ID: repeated targets must
    remain ambiguous in offline lineage matching.
    """
    if len(targets)<15 or not all(math.isfinite(value) for value in targets[:15]):
        raise ValueError('15 finite leg/waist targets required')
    return hashlib.sha256(struct.pack('<15f',*targets[:15])).hexdigest()


def dds_delivery_timing(source_ns,received_ns,callback_ros):
    """DDS publication/receipt timing, not sensor acquisition or motor acknowledgement.

    Valid only for comparable system clocks. Missing/zero/future timestamps
    remain unavailable rather than reporting zero transport/queue latency.
    """
    output = {}
    for label,stamp in (('source',source_ns),('receive',received_ns)):
        if isinstance(stamp,(int,float)) and math.isfinite(stamp) and stamp>0:
            seconds = stamp*1e-9
            if math.isfinite(callback_ros) and seconds<=callback_ros:
                output[f'lowstate_dds_{label}_stamp_s'] = seconds
                output[f'lowstate_dds_{label}_to_callback_ms'] = 1000.*(callback_ros-seconds)
    return output


class DeviceTickFreshness:
    """Receipt-time watchdog for advancing device observations, not source age.

    Repeated/reordered packets cannot renew the progress lease. The near-zero
    restart convention matches DeviceTickCadence; uint32 wrap is normal progress.
    This cannot detect an advancing but consistently delayed stream.
    """
    def __init__(self):
        self.last = self.progress_at = None
        self.duplicates = self.reordered = self.resets = 0

    def observe(self,tick,now):
        tick = int(tick)
        if not math.isfinite(now) or not 0<=tick<=0xffffffff:
            return False
        if self.last is not None:
            delta = (tick-self.last)&0xffffffff
            if delta==0:
                self.duplicates += 1
                return False
            if delta>0x7fffffff:
                if not (tick<=100 and self.last>1000):
                    self.reordered += 1
                    return False
                self.resets += 1
        self.last,self.progress_at = tick,now
        return True

    def age(self,now):
        if self.progress_at is None or not math.isfinite(now) or now<self.progress_at:
            return math.inf
        return now-self.progress_at


class DeviceTickCadence:
    """Minimum-spaced observation cadence for a uint32 millisecond device clock.

    Wall-clock watchdogs remain separate. Duplicate/reordered ticks cannot add
    history frames; a device restart near zero starts a new inference epoch.
    Late observations rebase the next deadline rather than compressing history
    with a catch-up sample just before the next nominal phase boundary.
    """
    def __init__(self,period_s):
        if not math.isfinite(period_s) or period_s<=0.:
            raise ValueError('positive finite policy period required')
        self.period_ms = period_s*1000.
        self.last = None
        self.elapsed = self.sample = self.next = 0.
        self.missed = self.resets = 0

    def due(self,tick):
        if self.last is None:
            return True
        delta = (int(tick)-self.last)&0xffffffff
        if delta>0x7fffffff:
            return int(tick)<=100 and self.last>1000
        return delta>0 and self.elapsed+delta>=self.next

    def take(self,tick):
        tick = int(tick)
        reset = False
        if self.last is None:
            self.last = tick
            self.next = self.period_ms
            return self.period_ms/1000.,False
        delta = (tick-self.last)&0xffffffff
        if delta==0:
            return None
        if delta>0x7fffffff:
            if not (tick<=100 and self.last>1000):
                return None
            self.resets += 1
            self.last,self.elapsed,self.sample,self.next = tick,0.,0.,self.period_ms
            return self.period_ms/1000.,True
        self.last = tick
        self.elapsed += delta
        if self.elapsed<self.next:
            return None
        dt = (self.elapsed-self.sample)/1000.
        skipped = max(0,int((self.elapsed-self.next)//self.period_ms))
        self.missed += skipped
        self.next = self.elapsed+self.period_ms
        self.sample = self.elapsed
        return dt,reset


def result_is_stale(result,now,deadline):
    """Completion before a deadline does not permit arbitrarily late application."""
    age = now-result.submitted_at
    return not math.isfinite(age) or age<0. or age>deadline


def inference_timing(result,context,published_monotonic,published_ros):
    ordered = (all(math.isfinite(value) for value in
                   (result.submitted_at,result.completed_at,published_monotonic,published_ros)) and
               result.submitted_at <= result.completed_at <= published_monotonic)
    matched = context is not None and context.get('key') == (result.generation,result.sequence)
    output = dict(schema_version=1,role='policy_joint_target_publication_not_actuator_ack',
                  generation=result.generation,sequence=result.sequence,stamp=published_ros,
                  timing_valid=ordered,context_matched=matched,
                  submitted_monotonic_s=result.submitted_at,
                  completed_monotonic_s=result.completed_at,
                  published_monotonic_s=published_monotonic)
    if ordered:
        output.update(submit_to_complete_ms=1000.*(result.completed_at-result.submitted_at),
                      complete_to_publish_ms=1000.*(published_monotonic-result.completed_at),
                      submit_to_publish_ms=1000.*(published_monotonic-result.submitted_at))
    if matched:
        delivery = context.get('lowstate_dds_delivery',{})
        output.update(delivery)
        source_stamp = delivery.get('lowstate_dds_source_stamp_s')
        if source_stamp is not None and published_ros>=source_stamp:
            output['lowstate_dds_source_to_target_publish_ms'] = 1000.*(published_ros-source_stamp)
        for key in ('device_tick_ms','observation_dt_s','observation_clock','observation_filter_dt_s',
                    'observation_cadence','observation_period_s'):
            if key in context:
                output[key] = context[key]
        output['command_at_submit'] = context['command']
        age = published_ros-context['command_received_ros_s']
        output['command_receipt_age_at_target_publish_s'] = age if math.isfinite(age) and age>=0. else None
        lowstate_age = context['lowstate_receipt_age_at_submit_s']
        if ordered and math.isfinite(lowstate_age) and lowstate_age>=0.:
            output['lowstate_receipt_age_at_target_publish_s'] = lowstate_age+published_monotonic-result.submitted_at
    return output
