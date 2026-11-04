"""Continuous, positive piecewise-linear contact-pressure shear integral.

The frozen soil, load, geometry, cap and initial tangent are retained. Segment
resolution is a numerical choice; no whole-vehicle motion is used to choose it.
"""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np
from r5_literature_contact import reference


def kernel(z):
    """Integral (1-exp(-z*t)) against the two linear hat functions on [0,1]."""
    z=np.asarray(z,dtype=float)
    if np.any(z<0) or not np.isfinite(z).all():raise ValueError('Finite nonnegative kernel argument required')
    left=np.empty_like(z);right=np.empty_like(z);small=z<.25
    # Positive leading terms and stable alternating series near zero.
    a=z[small];term=a.copy();l=np.zeros_like(a);r=np.zeros_like(a)
    for n in range(1,18):
        l+=term/((n+1)*(n+2));r+=term/(n+2);term*=-a/(n+1)
    left[small]=l;right[small]=r
    a=z[~small];e=np.exp(-a);phi0=-np.expm1(-a)/a;phi1=(phi0-e)/a
    left[~small]=.5-phi0+phi1;right[~small]=.5-phi1
    return left,right


def ratio(lateral_ratio,law):
    x=np.asarray(lateral_ratio,dtype=float)
    if np.isnan(x).any():raise ValueError('NaN ratio')
    lam=abs(x);z=lam*law['rate_extent'];finite=np.isfinite(z)
    out=np.full_like(z,law['cap']);zz=z[finite]
    l,r=kernel(zz);value=law['cohesion_cap']*(l+r)
    p=np.asarray(law['pressure_density']);h=1/(len(p)-1);a,b=kernel(zz*h)
    pressure=np.zeros_like(zz)
    for i in range(len(p)-1):
        y=zz*(i*h);e=np.exp(-y);d=-np.expm1(-y)
        pressure+=h*((p[i]+p[i+1])*.5*d+e*(p[i]*a+p[i+1]*b))
    out[finite]=value+(law['cap']-law['cohesion_cap'])*pressure
    return np.sign(x)*out


def initial_slope(law):
    # Symmetric pressure and uniform cohesion have the same centroid 1/2.
    return law['cap']*law['rate_extent']/2


def compliance(lateral,rolling,law):
    lateral=np.asarray(lateral,dtype=float);rolling=np.asarray(rolling,dtype=float)
    if np.any(rolling<0) or not np.isfinite(lateral).all() or not np.isfinite(rolling).all():raise ValueError('Invalid physical speed')
    lateral,rolling=np.broadcast_arrays(lateral,rolling);v=abs(lateral)
    lam=np.divide(v,rolling,out=np.zeros_like(v),where=rolling>0)
    q=ratio(lam,law)
    c=np.divide(v,law['load_N']*q,out=np.zeros_like(v),where=q>0)
    tiny=(rolling>0)&(lam<1e-8);c[tiny]=rolling[tiny]/(law['load_N']*initial_slope(law))
    stopped=rolling==0;c[stopped]=v[stopped]/(law['load_N']*law['cap'])
    return c


def build(old,segments):
    if segments<4 or segments%2:raise ValueError('Even resolved segment count required')
    g,_=reference(old['load_N'],soil=old['soil'])
    entry=g['entry_angle_rad'];x=np.linspace(0,1,segments+1);u=entry*x
    effective=np.where(u<=entry/2,entry-u,u)
    pressure=old['soil']['Ks_Pa_per_m']*.076**old['soil']['n']*np.maximum(0,np.cos(effective)-math.cos(entry))**old['soil']['n']
    pressure=(pressure+pressure[::-1])/2
    pressure[0]=pressure[-1]=0.
    area=np.trapezoid(pressure,x)
    # This normalizes the discretized shape, not the frozen pressure force cap.
    density=pressure/area
    ccap=.076*.05/old['load_N']*old['soil']['cohesion_Pa']*entry
    return dict(load_N=old['load_N'],cap=old['effective_lateral_cap'],cohesion_cap=ccap,
        rate_extent=.076*entry/old['soil']['lateral_shear_length_m'],segments=segments,
        pressure_density=density.tolist(),entry_angle_rad=entry,sinkage_m=g['sinkage_m'],source_soil=old['soil'],
        pressure_shape_normalization='Unit integral; retains frozen pressure cap and symmetric centroid',
        leading_deficit_coefficient=.05*old['soil']['cohesion_Pa']*old['soil']['lateral_shear_length_m']/old['load_N'])


def write_law(path,law):
    fields=[law[k] for k in ['load_N','cap','cohesion_cap','rate_extent']]+[len(law['pressure_density'])]
    path.write_text('mooncar_piecewise_integral_v1\n'+' '.join(repr(x) for x in fields)+'\n'+' '.join(repr(x) for x in law['pressure_density'])+'\n')


def main():
    from scipy.integrate import quad
    p=argparse.ArgumentParser();p.add_argument('--previous-fit',required=True);p.add_argument('--output',required=True);args=p.parse_args()
    out=Path(args.output)
    if out.exists():raise FileExistsError('Preserve frozen integral profile')
    original=Path(args.previous_fit);old=json.loads(original.read_text())
    # Fixed numerical mesh-selection grid and thresholds, before any rover run.
    lambdas=np.unique(np.r_[np.logspace(-9,6,181),np.tan(np.radians(np.linspace(.25,88.99,357))),np.tan(np.radians([65,75,80,85,87.5,88]))])
    gates=dict(max_relative_force_error=1e-4,max_relative_derivative_error=1e-3)
    rows=[];selected=None
    refs={}
    for axle,o in old['axles'].items():
        base=build(o,4);entry=base['entry_angle_rad'];soil=o['soil'];T=base['rate_extent']
        def pressure(x,entry=entry,soil=soil):
            u=x*entry;eff=entry-u if x<=.5 else u
            return soil['Ks_Pa_per_m']*.076**soil['n']*max(0,math.cos(eff)-math.cos(entry))**soil['n']
        area=quad(pressure,0,.5,epsabs=1e-9)[0]+quad(pressure,.5,1,epsabs=1e-9)[0]
        def density(x,base=base,pressure=pressure,area=area):return base['cohesion_cap']+(base['cap']-base['cohesion_cap'])*pressure(x)/area
        def integral(lam,derivative=False,T=T,density=density,cap=base['cap']):
            z=T*lam
            if z<=40:
                fun=(lambda x:T*x*density(x)*math.exp(-z*x)) if derivative else (lambda x:density(x)*(-math.expm1(-z*x)))
                return quad(fun,0,.5,epsabs=1e-16,epsrel=1e-11)[0]+quad(fun,.5,1,epsabs=1e-16,epsrel=1e-11)[0]
            # Resolve the thin entry boundary layer with v=z*x.
            limit=min(z,100.);split=min(z*.5,limit)
            fun=(lambda v:T/z**2*v*density(v/z)*math.exp(-v)) if derivative else (lambda v:density(v/z)*math.exp(-v)/z)
            value=quad(fun,0,split,epsabs=1e-20,epsrel=1e-11)[0]+(quad(fun,split,limit,epsabs=1e-20,epsrel=1e-11)[0] if split<limit else 0)
            return value if derivative else cap-value
        refs[axle]=(np.array([integral(l) for l in lambdas]),np.array([integral(l,True) for l in lambdas]),integral)
    for segments in [8,16,32,64,128,256,512]:
        record=dict(segments=segments,axles={});laws={}
        for axle,o in old['axles'].items():
            law=build(o,segments);laws[axle]=law
            target,derivative,_=refs[axle];prediction=ratio(lambdas,law)
            # Five-point finite difference is diagnostic only; native ratio is analytic.
            delta=lambdas*1e-3
            tangent=(-ratio(lambdas+2*delta,law)+8*ratio(lambdas+delta,law)-8*ratio(lambdas-delta,law)+ratio(lambdas-2*delta,law))/(12*delta)
            reliable=derivative>1e-9
            rel=abs((prediction-target)/target);dr=abs((tangent[reliable]-derivative[reliable])/derivative[reliable])
            record['axles'][axle]=dict(max_relative_force_error=float(rel.max()),worst_lambda=float(lambdas[rel.argmax()]),max_relative_derivative_error=float(dr.max()))
        record['passed']=all(a['max_relative_force_error']<=gates['max_relative_force_error'] and a['max_relative_derivative_error']<=gates['max_relative_derivative_error'] for a in record['axles'].values())
        rows.append(record)
        if record['passed']:selected=laws;break
    if selected is None:raise RuntimeError('Segment resolution gate failed; no vehicle candidate generated')
    check_angles=[4,5,7.5,10,20,35,50,65,80,85,87.5,88]
    # These points include previously inspected angles: report as numerical validation.
    for axle,law in selected.items():
        target=refs[axle][2];law['initial_slope']=initial_slope(law)
        law['angle_validation']=[dict(beta_deg=b,reference_ratio=target(math.tan(math.radians(b))),integral_ratio=float(ratio(math.tan(math.radians(b)),law)),relative_error=float(abs(ratio(math.tan(math.radians(b)),law)/target(math.tan(math.radians(b)))-1))) for b in check_angles]
        law['curve_passed']=max(r['relative_error'] for r in law['angle_validation'])<=.01
    result=dict(kind='piecewise_linear_pressure_continuous_shear_integral',previous_fit_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
        no_empirical_parameters_fitted=True,whole_vehicle_data_used=False,production_changed=False,soil_caps_loads_unchanged=True,
        resolution_gates=gates,resolution_scan=rows,numerical_validation_not_new_angle_holdout=True,axles=selected)
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    for axle,law in selected.items():write_law(out.parent/(axle+'-law.txt'),law)
    print(json.dumps(dict(segments=next(iter(selected.values()))['segments'],resolution=rows[-1],angle_validation={k:v['angle_validation'] for k,v in selected.items()}),indent=2))


if __name__=='__main__':main()
