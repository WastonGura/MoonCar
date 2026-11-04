"""Pure, fail-closed quality policy for the opt-in laser odometry candidate.

This marks observations usable; it never replaces missing measurements or
constrains motion. Times are seconds: source stamps use ROS time and receipt
times use the consumer's monotonic wall clock.
"""
import math
import copy
import uuid


SCHEMA_VERSION = 1
REQUIRED_SUCCESSES = 3
MAX_OBSERVATION_AGE = .3
MAX_STATUS_WALL_AGE = .5
FRAME_LOOKAHEAD = .1
FUTURE_TOLERANCE = .1
_EPSILON = 1e-9


class LocalizationDispatchRejected(ValueError):
    """A fresh movement lease was refused by localization, never a permit."""
    def __init__(self, health):
        self.reason=health.get('reason','unknown')
        self.session_id=health.get('session_id')
        super().__init__('激光定位质量失效：'+str(self.reason))


def require_dispatch_ready(state, message):
    """Preserve all readiness gates while identifying localization refusal."""
    if state.get('ready'):return
    diagnostics=state.get('diagnostics',{});health=diagnostics.get('laser_localization',{})
    pose_error=diagnostics.get('pose_error','')
    if health.get('required') and (not health.get('healthy') or
            (state.get('pose') is None and pose_error.startswith('激光定位质量失效：'))):
        health=dict(health)
        if health.get('healthy'):
            health['reason']=pose_error.split('：',1)[1]
        raise LocalizationDispatchRejected(health)
    raise ValueError(message)


def _finite(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


class LaserQualitySource:
    """Track genuine fits, with a new identity for every producer process."""

    def __init__(self, session_id=None):
        self.session_id = session_id or uuid.uuid4().hex
        self.sequence = 0
        self.last_input_stamp = None
        self.accepted_stamp = None
        self.ready_since_stamp = None
        self.successful_streak = 0
        self.reason = 'initializing'
        self.fault = None
        self.last_snapshot_stamp = None

    def reject(self, reason):
        """Invalidate a non-scan input without inventing a scan timestamp."""
        self.successful_streak = 0
        self.ready_since_stamp = None
        self.reason = reason

    def observe(self, stamp, result):
        if self.fault:
            return
        if not _finite(stamp):
            self.reject('invalid_input_stamp')
            return
        if self.last_input_stamp is not None and stamp < self.last_input_stamp - _EPSILON:
            self.fault = 'clock_moved_backwards'
            self.reject(self.fault)
            return
        if self.last_input_stamp is not None and stamp <= self.last_input_stamp:
            self.reject('nonincreasing_scan_stamp')
            return
        self.last_input_stamp = float(stamp)
        if (not isinstance(result, dict) or type(result.get('accepted')) is not bool
                or not isinstance(result.get('reason'), str) or not result['reason']):
            self.reject('invalid_result')
            return
        if not result['accepted']:
            if result['reason'] != 'stationary_hold':
                self.reject(result['reason'])
            return
        if (self.accepted_stamp is None
                or stamp - self.accepted_stamp > MAX_OBSERVATION_AGE + _EPSILON):
            self.reject('recovering')
        self.accepted_stamp = float(stamp)
        # Initialization publishes an origin observation, but has not fitted a
        # new scan against that reference. Require three subsequent real fits.
        if result['reason'] == 'initialized':
            self.reject('initializing')
            return
        self.successful_streak = min(REQUIRED_SUCCESSES, self.successful_streak + 1)
        if self.successful_streak >= REQUIRED_SUCCESSES:
            if self.ready_since_stamp is None:
                self.ready_since_stamp = float(stamp)
            self.reason = 'healthy'
        else:
            self.reason = 'recovering'

    def snapshot(self, now_sim):
        if not _finite(now_sim):
            self.fault = 'invalid_clock'
            now_sim = self.last_snapshot_stamp if self.last_snapshot_stamp is not None else 0.
        if self.last_snapshot_stamp is not None and now_sim < self.last_snapshot_stamp - _EPSILON:
            self.fault = 'clock_moved_backwards'
        self.last_snapshot_stamp = float(now_sim)
        if self.fault:
            self.reject(self.fault)
        elif (self.accepted_stamp is not None
              and now_sim - self.accepted_stamp > MAX_OBSERVATION_AGE + _EPSILON):
            self.reject('observation_stale')
        healthy = self.ready_since_stamp is not None and not self.fault
        reason = self.reason
        if healthy and self.accepted_stamp > now_sim + FUTURE_TOLERANCE + _EPSILON:
            healthy = False
            reason = 'observation_from_future'
        self.sequence += 1
        return dict(schema_version=SCHEMA_VERSION, session_id=self.session_id,
                    sequence=self.sequence, stamp=float(now_sim),
                    accepted_stamp=self.accepted_stamp, ready_since_stamp=self.ready_since_stamp,
                    healthy=healthy, reason=reason, successful_streak=self.successful_streak)


class LocalizationQuality:
    """Validate producer health independently of fresh EKF/TF timestamps.

    A different producer identity or a backwards ROS clock latches a fault.
    Only an explicit new consumer session (reset) may clear those faults.
    """

    def __init__(self, required=False):
        self.required = bool(required)
        self.reset()

    def reset(self):
        self.revisit_history_epoch=getattr(self,'revisit_history_epoch',0)+1
        self._revisit_history=[];self._revisit_cursor=0;self._revisit_stamp=None
        self.revisit_health_epoch=0;self._revisit_previous_healthy=False
        self.session_id = None
        self.sequence = None
        self.payload = None
        self.received_wall = None
        self.last_sim = None
        self.fault = None
        self.invalid_reason = None
        self.recovery_required = False
        self.recovery_streak = 0
        self.recovery_after_stamp = None
        self.local_ready_since_stamp = None

    def _remember_revisit_receipt(self,answer,now_sim,now_wall):
        payload=self.payload
        if not isinstance(payload,dict) or payload.get('localization_backend')!='metric_rgbd_3d':return
        stamp=payload.get('accepted_stamp')
        if not _finite(stamp) or self._revisit_stamp is not None and stamp<=self._revisit_stamp:return
        fit=payload.get('last_fit')
        if not isinstance(fit,dict):fit={}
        std=fit.get('position_std_m');ref=fit.get('reference_id')
        valid=(fit.get('accepted') is True and fit.get('exposure_stamp',fit.get('stamp'))==stamp
               and type(ref) is int and ref>0 and isinstance(std,(list,tuple)) and len(std)==3
               and all(_finite(value) and value>=0 for value in std))
        # Only small source metadata is retained; no images, TF or pose is
        # reconstructed, and recording never changes a quality decision.
        selected=dict(accepted=True,exposure_stamp=stamp,reference_id=ref,position_std_m=list(std)) if valid else {}
        attempts=fit.get('reference_attempts')
        if isinstance(attempts,list) and len(attempts)<=6:
            selected['reference_attempts']=[dict(reference_id=item['reference_id'],reason=item['reason'],accepted=item['accepted'])
                for item in attempts if isinstance(item,dict) and type(item.get('reference_id')) is int
                and item['reference_id']>0 and isinstance(item.get('reason'),str) and len(item['reason'])<=128
                and type(item.get('accepted')) is bool]
        admission=fit.get('view_admission')
        adoption=admission.get('anchor_adoption') if isinstance(admission,dict) else None
        if isinstance(adoption,dict):
            event={key:adoption[key] for key in ('adopted_reference_id','selected_reference_id','additional_reference_id','retired_reference_id')
                   if type(adoption.get(key)) is int and adoption[key]>0}
            if isinstance(adoption.get('additional_reason'),str) and len(adoption['additional_reason'])<=128:
                event['additional_reason']=adoption['additional_reason']
            selected['view_admission']=dict(anchor_adoption=event)
        self._revisit_cursor+=1;self._revisit_stamp=stamp
        self._revisit_history.append(dict(cursor=self._revisit_cursor,
            payload=dict(localization_backend='metric_rgbd_3d',healthy=True,session_id=payload['session_id'],
                         sequence=payload['sequence'],accepted_stamp=stamp,last_fit=selected),
            health=dict(answer),consumer_health_epoch=self.revisit_health_epoch,
            checked_sim=now_sim,received_wall=self.received_wall,checked_wall=now_wall))
        self._revisit_history=self._revisit_history[-128:]

    def revisit_receipts(self,cursor=None):
        """Read under the consumer owner's existing lock; copies are detached."""
        after=0
        if cursor is not None:
            if (not isinstance(cursor,(list,tuple)) or len(cursor)!=2
                    or any(type(value) is not int for value in cursor)
                    or cursor[0]!=self.revisit_history_epoch or not 0<=cursor[1]<=self._revisit_cursor):
                raise ValueError('RGBD evidence history cursor belongs to another reset')
            after=cursor[1]
            if self._revisit_history and after<self._revisit_history[0]['cursor']-1:
                raise ValueError('RGBD evidence history cursor overflowed')
        return dict(cursor=(self.revisit_history_epoch,self._revisit_cursor),
                    consumer_health_epoch=self.revisit_health_epoch,
                    rows=copy.deepcopy([row for row in self._revisit_history if row['cursor']>after]))

    def _invalid_status(self, reason):
        self.invalid_reason = reason
        self.recovery_required = True
        self.recovery_streak = 0
        self.recovery_after_stamp = self.payload['accepted_stamp'] if self.payload else None
        self.local_ready_since_stamp = None

    @staticmethod
    def _valid(payload):
        if not isinstance(payload, dict):
            return False
        if (type(payload.get('schema_version')) is not int or payload['schema_version'] != SCHEMA_VERSION
                or not isinstance(payload.get('session_id'), str) or not 1 <= len(payload['session_id']) <= 128
                or type(payload.get('sequence')) is not int or payload['sequence'] < 1
                or not _finite(payload.get('stamp')) or type(payload.get('healthy')) is not bool
                or not isinstance(payload.get('reason'), str) or not payload['reason']
                or type(payload.get('successful_streak')) is not int
                or not 0 <= payload['successful_streak'] <= REQUIRED_SUCCESSES):
            return False
        for key in ('accepted_stamp', 'ready_since_stamp'):
            if key not in payload or (payload[key] is not None and not _finite(payload[key])):
                return False
        accepted, ready = payload['accepted_stamp'], payload['ready_since_stamp']
        if accepted is not None and accepted > payload['stamp'] + FUTURE_TOLERANCE + _EPSILON:
            return False
        if ready is not None and (accepted is None or ready > accepted + _EPSILON):
            return False
        if payload['healthy'] and (accepted is None or ready is None
                                   or payload['successful_streak'] < REQUIRED_SUCCESSES):
            return False
        return True

    def observe(self, payload, now_wall):
        if self.fault:
            return
        if not _finite(now_wall) or not self._valid(payload):
            self._invalid_status('invalid_status')
            return
        if self.session_id is not None and payload['session_id'] != self.session_id:
            self.fault = 'producer_session_changed'
            return
        if self.sequence is not None and payload['sequence'] <= self.sequence:
            self._invalid_status('nonincreasing_status_sequence')
            return
        epoch_changed = False
        if self.payload is not None:
            old_accepted = self.payload['accepted_stamp']
            old_ready, new_ready = self.payload['ready_since_stamp'], payload['ready_since_stamp']
            if (payload['stamp'] < self.payload['stamp'] - _EPSILON
                    or (old_accepted is not None and payload['accepted_stamp'] is not None
                        and payload['accepted_stamp'] < old_accepted - _EPSILON)):
                self.fault = 'clock_moved_backwards'
                return
            if old_ready is not None and new_ready is not None:
                if new_ready < old_ready - _EPSILON:
                    self.fault = 'recovery_epoch_moved_backwards'
                    return
                epoch_changed = new_ready > old_ready + _EPSILON
        self.session_id = payload['session_id']
        self.sequence = payload['sequence']
        self.payload = dict(payload)
        self.received_wall = float(now_wall)
        self.invalid_reason = None
        if self.recovery_required:
            accepted = payload['accepted_stamp']
            if not payload['healthy'] and payload['successful_streak'] == 0:
                self.recovery_streak = 0
                self.recovery_after_stamp = accepted
            elif (accepted is not None and (self.recovery_after_stamp is None
                                            or accepted > self.recovery_after_stamp + _EPSILON)):
                self.recovery_after_stamp = accepted
                self.recovery_streak += 1
                if self.recovery_streak >= REQUIRED_SUCCESSES and payload['healthy']:
                    self.recovery_required = False
                    self.local_ready_since_stamp = accepted
        # Even if an unhealthy message was lost, a changed ready epoch proves an
        # intervening outage. Consumers must observe a blocked check before a
        # later increasing heartbeat can re-enable new commands or captures.
        if epoch_changed:
            self.invalid_reason = 'recovery_epoch_changed'

    def check(self, now_sim, now_wall, observation_stamp=None):
        answer = dict(required=self.required, healthy=False, reason='status_missing',
                      session_id=self.session_id, sequence=self.sequence,
                      accepted_stamp=self.payload['accepted_stamp'] if self.payload else None,
                      ready_since_stamp=self.payload['ready_since_stamp'] if self.payload else None)

        def finish(reason, healthy=False):
            answer.update(reason=reason, healthy=healthy)
            if self._revisit_previous_healthy and not healthy:self.revisit_health_epoch+=1
            self._revisit_previous_healthy=healthy
            if healthy and self.required:self._remember_revisit_receipt(answer,now_sim,now_wall)
            return answer

        if not self.required:
            return finish('disabled', True)
        if not _finite(now_sim) or not _finite(now_wall):
            return finish('invalid_clock')
        if self.last_sim is not None and now_sim < self.last_sim - _EPSILON:
            self.fault = 'clock_moved_backwards'
        self.last_sim = float(now_sim)
        if self.fault:
            return finish(self.fault)
        if self.invalid_reason:
            return finish(self.invalid_reason)
        if self.payload is None:
            return finish('status_missing')
        wall_age = now_wall - self.received_wall
        answer['status_age_wall_seconds'] = wall_age
        if wall_age < -_EPSILON:
            return finish('invalid_wall_clock')
        if wall_age > MAX_STATUS_WALL_AGE + _EPSILON:
            if not self.recovery_required:
                self._invalid_status('status_stale')
            return finish('status_stale')
        if self.payload['stamp'] > now_sim + FUTURE_TOLERANCE + _EPSILON:
            return finish('status_from_future')
        if not self.payload['healthy']:
            return finish(self.payload['reason'])
        if self.recovery_required:
            return finish('status_recovering')
        accepted = self.payload['accepted_stamp']
        age = now_sim - accepted
        answer['observation_age_sim_seconds'] = age
        if age < -FUTURE_TOLERANCE - _EPSILON:
            return finish('observation_from_future')
        if age > MAX_OBSERVATION_AGE + _EPSILON:
            if not self.recovery_required:
                self._invalid_status('observation_stale')
            return finish('observation_stale')
        if observation_stamp is not None:
            if not _finite(observation_stamp):
                return finish('invalid_observation_stamp')
            ready = self.payload['ready_since_stamp']
            if self.local_ready_since_stamp is not None:
                ready = max(ready, self.local_ready_since_stamp)
            if observation_stamp < ready - _EPSILON:
                return finish('observation_before_ready')
            if observation_stamp > accepted + FRAME_LOOKAHEAD + _EPSILON:
                return finish('observation_after_measurement')
        return finish('healthy', True)
