"""Positive two-exponential reduction of the frozen literature reference.

This is a wheel-local constitutive candidate, not empirical soil calibration.
No vehicle-motion data are accepted by the fit interface.
"""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np
from r5_literature_contact import reference,gazebo_ratio

TRAIN_DEG=[.25,.5,1,1.5,2,2.5,3,15,30,45,60,75]
HOLDOUT_DEG=[4,5,7.5,10,20,35,50,65,80,85]

def ratio_from_lambda(lateral_ratio,cap,weight,rate_fast,rate_slow):
    if not (0<weight<1 and 0<rate_slow<=rate_fast and cap>0):
        raise ValueError('Positive ordered mixture parameters required')
    x=np.asarray(lateral_ratio,dtype=float)
    if np.isnan(x).any():raise ValueError('NaN lateral ratio')
    y=cap*(weight*(-np.expm1(-rate_fast*abs(x)))+(1-weight)*(-np.expm1(-rate_slow*abs(x))))
    return np.sign(x)*y

def native_compliance(lateral_speed,rolling_speed,normal_force,cap,weight,rate_fast,rate_slow):
    """Secant FDS coefficient. Rolling speed is absolute radius*joint speed.

    Zero rolling speed uses the continuous saturated-force limit; no arbitrary
    speed floor is introduced. Inactive pure-turn gating is applied outside.
    """
    if not all(math.isfinite(v) for v in (lateral_speed,rolling_speed,normal_force)) or rolling_speed<0 or normal_force<=0:
        raise ValueError('Invalid local physical state')
    slope=cap*(weight*rate_fast+(1-weight)*rate_slow)
    if not (0<weight<1 and 0<rate_slow<=rate_fast and cap>0):raise ValueError('Invalid mixture')
    if rolling_speed==0:return abs(lateral_speed)/(normal_force*cap)
    lam=abs(lateral_speed)/rolling_speed
    if lam<1e-8:return rolling_speed/(normal_force*slope)
    force_ratio=float(ratio_from_lambda(lam,cap,weight,rate_fast,rate_slow))
    return abs(lateral_speed)/(normal_force*force_ratio)

def fit_axle(old):
    from scipy.optimize import least_squares
    _,target=reference(old['load_N'],soil=old['soil'])
    cap=old['effective_lateral_cap'];train=np.asarray(TRAIN_DEG)
    lam=np.tan(np.radians(train));truth=target(train)
    initial_slope=float(target(np.degrees(np.arctan(1e-7))))/1e-7
    mean_rate=initial_slope/cap
    # Fit only declared training angles, including its initial tangent.
    def unpack(x):
        w=1/(1+np.exp(-x[0]));a,b=np.exp(x[1:])
        return (w,a,b) if a>=b else (1-w,b,a)
    def residual(x):
        w,a,b=unpack(x)
        values=ratio_from_lambda(lam,cap,w,a,b)
        tangent=cap*(w*a+(1-w)*b)
        return np.r_[(values-truth)/truth,(tangent-initial_slope)/initial_slope]
    trials=[]
    for w in (.25,.5,.75):
        for slow in (.1,.5):
            x=[math.log(w/(1-w)),math.log(mean_rate*2),math.log(mean_rate*slow)]
            result=least_squares(residual,x,bounds=([-12,-12,-12],[12,5,5]),
                xtol=1e-13,ftol=1e-13,gtol=1e-13,max_nfev=5000)
            trials.append(result)
    best=min(trials,key=lambda r:float(np.dot(r.fun,r.fun)))
    w,a,b=unpack(best.x)
    def rows(angles):
        values=target(angles);pred=ratio_from_lambda(np.tan(np.radians(angles)),cap,w,a,b)
        linear=gazebo_ratio(angles,old['compliance_lateral'],cap)
        return [dict(beta_deg=float(angle),reference_ratio=float(y),nonlinear_ratio=float(p),
            nonlinear_relative_error=float(abs(p-y)/abs(y)),linear_relative_error=float(abs(l-y)/abs(y)))
            for angle,y,p,l in zip(angles,values,pred,linear)]
    train_rows=rows(TRAIN_DEG);holdout_rows=rows(HOLDOUT_DEG)
    return dict(load_N=old['load_N'],cap=cap,weight=float(w),rate_fast=float(a),rate_slow=float(b),
        exact_reference_initial_slope=initial_slope,fitted_initial_slope=float(cap*(w*a+(1-w)*b)),
        training=train_rows,angle_holdout=holdout_rows,
        fit_passed=max(r['nonlinear_relative_error'] for r in train_rows)<=.01,
        holdout_passed=max(r['nonlinear_relative_error'] for r in holdout_rows)<=.01,
        optimizer_success=bool(best.success),training_cost=float(np.dot(best.fun,best.fun)),
        source_soil=old['soil'],hardware_calibrated=False)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--previous-fit',required=True);p.add_argument('--output',required=True);args=p.parse_args()
    output=Path(args.output)
    if output.exists():raise FileExistsError('Preserve frozen candidate evidence')
    old_path=Path(args.previous_fit);old=json.loads(old_path.read_text())
    result=dict(kind='nonlinear_literature_model_projection',family='positive_two_exponential_mixture',
        equation='q(lambda)=sign(lambda)*mu*[w*(1-exp(-a*abs(lambda)))+(1-w)*(1-exp(-b*abs(lambda)))]',
        previous_fit_sha256=hashlib.sha256(old_path.read_bytes()).hexdigest(),
        training_deg=TRAIN_DEG,holdout_deg=HOLDOUT_DEG,relative_error_gate=.01,
        whole_vehicle_data_used=False,production_changed=False,
        axles={name:fit_axle(axle) for name,axle in old['axles'].items()})
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({name:{k:v for k,v in x.items() if k not in ('training','angle_holdout','source_soil')} for name,x in result['axles'].items()},indent=2))

if __name__=='__main__':main()
