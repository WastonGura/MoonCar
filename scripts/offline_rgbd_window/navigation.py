"""Publication checks for a window pose; no prediction, reset or legacy fallback."""
import numpy as np

from localization_quality import MAX_OBSERVATION_AGE, MAX_STATUS_WALL_AGE, FUTURE_TOLERANCE


class WindowOutputGate:
    def __init__(self):
        self.last_stamp_ns=None
        self.gauge_stamp_ns=None

    def accept(self,row,sensor_stamp_ns,now_ns,wall_ns,source_fault=None):
        if source_fault:return None,'source_fault'
        if row.get('output') is not True:return None,'no_pose'
        stamp=row.get('stamp_ns');gauge=row.get('gauge_stamp_ns')
        if type(stamp) is not int or stamp!=sensor_stamp_ns:return None,'exposure_mismatch'
        if self.last_stamp_ns is not None and stamp<=self.last_stamp_ns:return None,'nonincreasing_pose'
        if type(gauge) is not int or gauge>stamp:return None,'invalid_gauge'
        if self.gauge_stamp_ns is not None and gauge!=self.gauge_stamp_ns:return None,'gauge_changed'
        age=(now_ns-stamp)/1e9
        received=row.get('received_wall_ns')
        if age>MAX_OBSERVATION_AGE+1e-9 or age < -FUTURE_TOLERANCE-1e-9:return None,'pose_age'
        if type(received) is not int or not 0<=wall_ns-received<=MAX_STATUS_WALL_AGE*1e9:return None,'wall_age'
        cache_age=row.get('cache_age_s')
        if type(cache_age) not in (int,float) or not np.isfinite(cache_age) or not 0<=cache_age<=1.:return None,'cache_age'
        try:
            xyz=np.array(row['position'],dtype=float,copy=True)
            C=np.array(row['covariance_conditional'],dtype=float,copy=True)
        except (KeyError,TypeError,ValueError):return None,'invalid_pose'
        if (xyz.shape!=(3,) or C.shape!=(3,3) or not np.isfinite(xyz).all() or not np.isfinite(C).all()
                or not np.allclose(C,C.T,atol=1e-12,rtol=1e-8) or np.linalg.eigvalsh(C).min() < -1e-12):
            return None,'invalid_pose'
        self.last_stamp_ns=stamp;self.gauge_stamp_ns=gauge
        return dict(xyz=xyz,covariance=C,stamp=stamp/1e9,accepted=True,reason='accepted'),'accepted'
