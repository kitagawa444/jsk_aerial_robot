"""ROS-independent command-to-measured-force identification utilities.

Model: tau * dy/dt + y = gain * u(t-delay) + offset.
The measured output may include observer/filter dynamics. This is not a model
of true contact force unless that output was independently measured.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter


def ordered_unique_samples(rows):
    """Deduplicate paused-clock samples; never hide clock resets by sorting."""
    unique={};previous=None
    for row in rows:
        if not np.isfinite(row[0]) or (previous is not None and row[0]<previous):
            raise ValueError('Source clock moved backwards; split the bag at the reset')
        previous=row[0];unique[row[0]]=row
    return list(unique.values())


def causal_sample(times,values,query,max_age=.15):
    times=np.asarray(times,dtype=float);values=np.asarray(values,dtype=float)
    if len(times)<2 or not np.all(np.diff(times)>0):raise ValueError('Source timestamps must increase')
    index=np.searchsorted(times,query,side='right')-1
    valid=(index>=0)
    index=np.maximum(index,0)
    valid&=(query-times[index]<=max_age)
    return values[index],valid


def response_basis(time,u,tau,delay):
    dt=float(np.median(np.diff(time)))
    a=np.exp(-dt/tau)
    delayed=np.interp(time-delay,time,u,left=u[0])
    response=lfilter([0.,1-a],[1.,-a],delayed)
    return np.column_stack([response,np.ones(len(time)),np.exp(-(time-time[0])/tau)])


def fit_response(time,u,y,validation_start,minimum_range=.08):
    """Fit on the first interval, free-run across held-out data without refitting.

    Gain/offset/initial state are linear parameters. Tau and delay are estimated
    with multiple starts. The returned acceptance is empirical and local to this
    experiment; uncertainty below is a local least-squares diagnostic, not a
    hardware safety guarantee.
    """
    time=np.asarray(time,dtype=float);u=np.asarray(u,dtype=float);y=np.asarray(y,dtype=float)
    if time.ndim!=1 or not (time.shape==u.shape==y.shape) or len(time)<100:
        raise ValueError('Need at least 100 aligned scalar samples')
    if not np.all(np.isfinite(np.r_[time,u,y])) or not np.all(np.diff(time)>0):
        raise ValueError('Nonfinite values or nonincreasing timestamps')
    dt=float(np.median(np.diff(time)))
    if np.max(abs(np.diff(time)-dt))>1e-6:raise ValueError('Fit requires a regular grid')
    train=time<validation_start;test=~train
    if train.sum()<100 or test.sum()<100:raise ValueError('Separate training and validation intervals required')
    span=float(np.percentile(u[train],95)-np.percentile(u[train],5))
    if span<minimum_range:
        return {'accepted':False,'reasons':['insufficient_input_excitation'],
                'input_range_N':span,'samples':len(time)},None
    u0=float(np.median(u[:max(1,int(.5/dt))]));centered=u-u0
    max_tau=min(3.,(time[train][-1]-time[0])/4)
    def solve(parameters):
        tau=np.exp(parameters[0]);delay=parameters[1]
        matrix=response_basis(time,centered,tau,delay)
        coefficients=np.linalg.lstsq(matrix[train],y[train],rcond=None)[0]
        return matrix,coefficients
    def residual(parameters):
        matrix,coefficients=solve(parameters)
        return (matrix@coefficients-y)[train]
    candidates=[]
    for tau in np.unique(np.clip([.04,.15,.4,1.],max(.005,dt/2),max_tau)):
        for delay in [0.,.04,.15,.35]:
            fit=least_squares(residual,[np.log(tau),delay],
                bounds=([np.log(max(.005,dt/2)),0.],[np.log(max_tau),.6]),
                max_nfev=120,xtol=1e-9,ftol=1e-9,gtol=1e-9)
            candidates.append(fit)
    best=min(candidates,key=lambda fit:np.sum(fit.fun**2))
    tau,delay=float(np.exp(best.x[0])),float(best.x[1])
    matrix,coefficients=solve(best.x)
    predicted=matrix@coefficients
    gain,center_offset,initial=map(float,coefficients)
    def score(mask):
        error=y[mask]-predicted[mask];norm=float(np.linalg.norm(y[mask]-y[mask].mean()))
        return {'rmse_N':float(np.sqrt(np.mean(error**2))),
                'fit_percent':float(100*(1-np.linalg.norm(error)/norm)) if norm>1e-9 else None,
                'output_range_N':float(np.percentile(y[mask],95)-np.percentile(y[mask],5))}
    training,validation=score(train),score(test)
    # Local uncertainty includes gain, offset and initial-condition nuisance
    # parameters, instead of treating them as perfectly known.
    params=np.r_[tau,delay,coefficients]
    def predict(p):return response_basis(time,centered,p[0],p[1])@p[2:]
    jac=[]
    for j in range(5):
        step=1e-5*max(1,abs(params[j]));high=params.copy();low=params.copy()
        high[j]+=step;low[j]-=step
        jac.append(((predict(high)-predict(low))/(2*step))[train])
    jac=np.array(jac).T
    variance=float(np.sum((predicted[train]-y[train])**2)/max(1,train.sum()-5))
    covariance=variance*np.linalg.pinv(jac.T@jac,rcond=1e-12)
    sigma=np.sqrt(np.maximum(0,np.diag(covariance)))
    correlation=float(covariance[0,1]/max(1e-20,sigma[0]*sigma[1]))
    reasons=[]
    if not best.success:reasons.append('fit_did_not_converge')
    if not .2<=gain<=2:reasons.append('implausible_or_unidentified_gain')
    if validation['fit_percent'] is None or validation['fit_percent']<60:reasons.append('poor_held_out_prediction')
    if tau>max_tau*.98 or delay>.59:reasons.append('parameter_at_search_boundary')
    if tau<dt:reasons.append('time_constant_below_sampling_resolution')
    if abs(correlation)>.98:reasons.append('delay_time_constant_confounded')
    if 1.96*sigma[0]>tau*.5:reasons.append('time_constant_uncertain')
    if delay<dt:delay_label='below_one_sample_resolution'
    else:delay_label='resolved_within_model'
    result={'accepted':not reasons,'reasons':reasons,'model':'first_order_plus_dead_time',
        'equation':'tau*dy/dt + y = gain*u(t-delay) + offset',
        'gain':gain,'tau_s':tau,'delay_s':delay,'delay_resolution':delay_label,
        'offset_N':center_offset-gain*u0,'initial_output_N':center_offset+initial,
        'model_t90_s':float(delay+np.log(10)*tau),'model_t95_s':float(delay+np.log(20)*tau),
        'sample_period_s':dt,'input_range_N':span,'training':training,'validation':validation,
        'samples':len(time),'training_samples':int(train.sum()),'validation_samples':int(test.sum()),
        'local_parameter_standard_errors':dict(zip(['tau_s','delay_s','gain','centered_offset_N','initial_term_N'],map(float,sigma))),
        'tau_delay_correlation':correlation,
        'uncertainty_scope':'Local independent-residual least-squares diagnostic; not a certified confidence or robustness bound.',
        'u_center_N':u0,'linear_coefficients_centered':coefficients.tolist()}
    return result,predicted
