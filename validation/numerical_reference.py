"""Independent NumPy/SciPy equation cross-check, NOT execution of MATLAB/Simulink.
Run: python validation/numerical_reference.py
Dependencies: numpy, scipy, matplotlib. Outputs stay in validation/.
This duplicates equations intentionally; MATLAB tests exercise the actual .m files.
"""
from pathlib import Path
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def vee(M):
    return M[[2, 0, 1], [1, 2, 0]]


def cross(a, b):
    return np.cross(a, b, axisa=0, axisb=0, axisc=0)


def unit(r, rd, rdd):
    L = np.linalg.norm(r)
    n = r/L
    Ld = n @ rd
    nd = (rd-n*Ld)/L
    ndd = (rdd-n*(n@rdd))/L - 2*Ld/L*nd - n*(nd@nd)
    return n, nd, ndd


def config(N=3, mode='paper', wind=False):
    angles = 2*np.pi*np.arange(N)/N
    rho = .8*np.array([np.cos(angles), np.sin(angles), np.zeros(N)])
    edges = [(i, j) for i in range(N) for j in range(i+1, N)]
    arm = .225/np.sqrt(2)
    B = np.array([[1., 1., 1., 1.], [arm, arm, -arm, -arm],
                  [-arm, arm, arm, -arm], [.015, -.015, .015, -.015]])
    return dict(N=N, mode=mode, wind=wind, rho=rho, edges=edges,
                lengths=np.array([np.linalg.norm(rho[:, i]-rho[:, j]) for i,j in edges]),
                B=B, Bi=np.linalg.inv(B), I=np.array([.015, .015, .025]),
                receiver=np.array([4., -3., 15.]), wavelength=299792458/138e6)


def reference(t, c):
    w=2*np.pi/40
    if t < 5:
        u=t/5
        ph=w*5*(2.5*u**4-3*u**5+u**6)
        pd=w*(10*u**3-15*u**4+6*u**5)
        pdd=w/5*(30*u**2-60*u**3+30*u**4)
    else:
        ph=w*(t-2.5); pd=w; pdd=0.
    f=np.array([2.5*np.sin(ph), 1.5*np.sin(2*ph), .35*np.sin(ph)])
    fp=np.array([2.5*np.cos(ph), 3*np.cos(2*ph), .35*np.cos(ph)])
    fpp=np.array([-2.5*np.sin(ph), -6*np.sin(2*ph), -.35*np.sin(ph)])
    pc=np.array([0., 0., 3.])+f; cv=fp*pd; ca=fpp*pd**2+fp*pdd
    n, nd, ndd=unit(c['receiver']-pc, -cv, -ca)
    seed=np.array([1.,0.,0.]); ss=seed-n*n[0]
    sd=-nd*n[0]-n*nd[0]; sdd=-ndd*n[0]-2*nd*nd[0]-n*ndd[0]
    b, bd, bdd=unit(ss, sd, sdd)
    b2=cross(n,b); b2d=cross(nd,b)+cross(n,bd)
    b2dd=cross(ndd,b)+2*cross(nd,bd)+cross(n,bdd)
    R=np.column_stack((b,b2,n)); Rd=np.column_stack((bd,b2d,nd))
    Rdd=np.column_stack((bdd,b2dd,ndd))
    W=Rd@R.T; Wd=Rdd@R.T+Rd@Rd.T
    return dict(c=pc, cv=cv, ca=ca, R=R, Rd=Rd, Rdd=Rdd,
                omega=vee((W-W.T)/2), omegad=vee((Wd-Wd.T)/2),
                p=pc[:,None]+R@c['rho'], v=cv[:,None]+Rd@c['rho'],
                a=ca[:,None]+Rdd@c['rho'])


def pose(P, V, c):
    s=P-P.mean(axis=1)[:,None]; vr=V-V.mean(axis=1)[:,None]
    A=s@c['rho'].T; U, d, Vt=np.linalg.svd(A)
    R=U@np.diag([1.,1.,np.linalg.det(U@Vt)])@Vt
    S=R.T@A; S=(S+S.T)/2; K=R.T@(vr@c['rho'].T)
    om=R@np.linalg.solve(np.trace(S)*np.eye(3)-S,vee(K-K.T))
    return R, om, s, vr


def formation(t,X,c):
    r=reference(t,c); P=X[:3]; V=X[3:6]
    if c['mode']=='geometric':
        return r['a']+np.array([2.5,2.5,3.5])[:,None]*(r['p']-P)+np.array([2.8,2.8,3.2])[:,None]*(r['v']-V)
    R, wa, s, vr=pose(P,V,c); E=R@r['R'].T
    e=Rotation.from_matrix(E).as_rotvec(); ang=np.linalg.norm(e); S=skew(e)
    b=1/12+ang**2/720 if ang<1e-5 else (1-.5*ang/np.tan(.5*ang))/ang**2
    ed=(np.eye(3)-.5*S+b*S@S)@(wa-E@r['omega'])
    wc=r['omega']-1.3*e; wcd=r['omegad']-1.3*ed
    vc=r['cv']-.9*(P.mean(axis=1)-r['c'])
    ac=r['ca']-.9*(V.mean(axis=1)-r['cv'])
    vd=vc[:,None]+cross(wc[:,None],s)
    vdd=ac[:,None]+cross(wcd[:,None],s)+cross(wc[:,None],vr)
    J=np.zeros((len(c['edges']),3*c['N'])); Jd=np.zeros_like(J); q=np.zeros(len(c['edges']))
    for k,(i,j) in enumerate(c['edges']):
        d=P[:,i]-P[:,j]; dv=V[:,i]-V[:,j]
        J[k,3*i:3*i+3]=d; J[k,3*j:3*j+3]=-d
        Jd[k,3*i:3*i+3]=dv; Jd[k,3*j:3*j+3]=-dv
        q[k]=d@d-c['lengths'][k]**2
    scale=3/c['N']; kv=.45*scale
    v=V.ravel(order='F'); vf=vd.ravel(order='F')-kv*J.T@q
    a=(-3.2*(v-vf)-scale*J.T@q-kv*Jd.T@q-2*kv*J.T@J@v+vdd.ravel(order='F')).reshape((3,c['N']),order='F')
    if c['N']>3:
        n=R[:,2]; nd=cross(wa,n)
        a-=n[:,None]*(4*(n@s)+3*(n@vr+nd@s))[None,:]
    return a


def quatR(q):
    q=q/np.linalg.norm(q); w=q[0]; v=q[1:]
    return (w*w-v@v)*np.eye(3)+2*np.outer(v,v)+2*w*skew(v)


def autopilot(a,x,c):
    raw=a.copy(); a=a.copy(); a[2]=np.clip(a[2],-4,4)
    h=np.linalg.norm(a[:2]); mh=min(4.,(9.81+a[2])*np.tan(np.deg2rad(35)))
    if h>mh: a[:2]*=mh/h
    F=a+np.array([0,0,9.81]); z=F/np.linalg.norm(F)
    y=cross(z,np.array([1.,0.,0.])); y/=np.linalg.norm(y)
    Rc=np.column_stack((cross(y,z),y,z)); R=quatR(x[6:10]); w=x[10:13]
    er=.5*vee(Rc.T@R-R.T@Rc)
    tau=-np.array([2.2,2.2,1.2])*er-np.array([.32,.32,.28])*w+cross(w,c['I']*w)
    thrust=max(0,F@R[:,2]); fr=c['Bi']@np.r_[thrust,tau]
    f=np.clip(fr,0,1.8e-5*620**2)
    return np.sqrt(f/1.8e-5), np.array([np.linalg.norm(a-raw)>1e-9,np.any(np.abs(fr-f)>1e-9)])


def rhs(t,X,U,c):
    q=X[6:10]; qn=q/np.linalg.norm(q,axis=0); qw,qx,qy,qz=qn
    w=X[10:13]; rotor=np.clip(X[13:17],0,620); f=1.8e-5*rotor**2
    wr=c['B']@f
    z=np.array([2*(qx*qz+qw*qy),2*(qy*qz-qw*qx),1-2*(qx*qx+qy*qy)])
    air=np.zeros(3)
    if c['wind']:
        air=np.array([.8,.3,0])+np.array([.3,.2,.1])*np.sin(np.array([.7*t,.53*t+.4,.31*t]))
    acc=z*wr[0]-np.array([.12,.12,.18])[:,None]*(X[3:6]-air[:,None])-np.array([0,0,9.81])[:,None]
    qd=.5*np.vstack((-np.sum(q[1:]*w,axis=0),q[0]*w+cross(q[1:],w)))
    qd+=(1-np.sum(q*q,axis=0))*q
    wd=(wr[1:]-cross(w,c['I'][:,None]*w)-np.array([.002,.002,.003])[:,None]*w)/c['I'][:,None]
    return np.vstack((X[3:6],acc,qd,wd,(U-X[13:17])/.025))


def checks():
    c=config(); worst_v=0.; worst_a=0.; worst_rate=0.
    for t in [0.2,2.,4.9,5.1,15.,35.,59.]:
        h=1e-4; r=reference(t,c); rp=reference(t+h,c); rm=reference(t-h,c)
        worst_v=max(worst_v,np.max(np.abs((rp['p']-rm['p'])/(2*h)-r['v'])))
        worst_a=max(worst_a,np.max(np.abs((rp['v']-rm['v'])/(2*h)-r['a'])))
        assert np.linalg.norm(r['R'].T@r['R']-np.eye(3))<1e-12
        # Polar derivative including deformation, independently finite-differenced.
        P=r['p'].copy(); P[:,0]+=[.03,-.02,.04]; V=r['v'].copy(); V[:,1]+=[.01,.02,-.03]
        R,w,_,_=pose(P,V,c); Rplus=pose(P+h*V,V,c)[0]; Rminus=pose(P-h*V,V,c)[0]
        rate=(Rplus-Rminus)/(2*h)
        worst_rate=max(worst_rate,np.max(np.abs(rate-skew(w)@R)))
    assert worst_v<1e-7 and worst_a<1e-7 and worst_rate<1e-7
    return dict(reference_velocity_max_error=worst_v, reference_acceleration_max_error=worst_a,
                polar_rotation_derivative_max_error=worst_rate)


def simulate(N=3,mode='paper',wind=False,T=60.,dt=.002):
    c=config(N,mode,wind); r=reference(0,c)
    X=np.zeros((17,N)); Rerr=Rotation.from_rotvec([0,np.deg2rad(12),0]).as_matrix()
    X[:3]=r['c'][:,None]+np.array([.15,-.1,.1])[:,None]+Rerr@r['R']@c['rho']
    X[6]=1.; X[13:17]=np.sqrt(9.81/(4*1.8e-5))
    U=np.zeros((4,N)); D=np.zeros((2,N)); records=[]; states=[]
    steps=round(T/dt); stride=round(.01/dt); logstride=round(.02/dt)
    start=time.time()
    for k in range(steps+1):
        t=k*dt
        if k%stride==0:
            a=formation(t,X,c)
            for i in range(N): U[:,i],D[:,i]=autopilot(a[:,i],X[:,i],c)
        if k%logstride==0:
            r=reference(t,c); P=X[:3]; V=X[3:6]; R,_,rel,_=pose(P,V,c)
            pc=P.mean(axis=1); los=c['receiver']-pc; los/=np.linalg.norm(los)
            pe=np.degrees(np.arccos(np.clip(R[:,2]@los,-1,1)))
            distances=np.array([np.linalg.norm(P[:,i]-P[:,j]) for i,j in c['edges']])
            de=distances-c['lengths']; tilt=np.degrees(np.arccos(np.clip([quatR(X[6:10,i])[2,2] for i in range(N)],-1,1)))
            ranges=np.linalg.norm(c['receiver'][:,None]-P,axis=0)
            amp=np.sqrt(.1)*c['wavelength']/(4*np.pi*ranges)
            efficiency=abs(np.sum(amp*np.exp(-2j*np.pi*ranges/c['wavelength'])))**2/np.sum(amp)**2
            records.append([t,np.linalg.norm(pc-r['c']),pe,np.sqrt(np.mean(de**2)),max(abs(de)),min(distances),
                            max(tilt),np.mean(D[0]),np.mean(D[1]),max(abs(np.linalg.norm(X[6:10],axis=0)-1)),
                            np.sqrt(np.mean((R[:,2]@rel)**2)),-10*np.log10(efficiency),
                            np.sqrt(np.mean(np.sum((P-r['p'])**2,axis=0)))])
            states.append(X.copy())
        if k==steps: break
        a1=rhs(t,X,U,c); a2=rhs(t+dt/2,X+dt/2*a1,U,c)
        a3=rhs(t+dt/2,X+dt/2*a2,U,c); a4=rhs(t+dt,X+dt*a3,U,c)
        X+=dt/6*(a1+2*a2+2*a3+a4)
    Z=np.array(records); XX=np.array(states); steady=Z[:,0]>=10
    summary=dict(N=N,mode=mode,wind=wind,duration_s=T,step_s=dt,
        centroid_RMSE_m=float(np.sqrt(np.mean(Z[:,1]**2))),pointing_RMSE_deg=float(np.sqrt(np.mean(Z[:,2]**2))),
        settled_pointing_RMSE_deg=float(np.sqrt(np.mean(Z[steady,2]**2))),
        edge_RMSE_m=float(np.sqrt(np.mean(Z[:,3]**2))),max_edge_error_m=float(max(Z[:,4])),
        min_separation_m=float(min(Z[:,5])),max_tilt_deg=float(max(Z[:,6])),
        accel_limited_percent=float(100*np.mean(Z[:,7])),motor_saturated_percent=float(100*np.mean(Z[:,8])),
        max_quaternion_norm_error=float(max(Z[:,9])),max_planarity_error_m=float(max(Z[:,10])),
        mean_coherent_loss_dB=float(np.mean(Z[:,11])),position_RMSE_m=float(np.sqrt(np.mean(Z[:,12]**2))),
        elapsed_wall_seconds=time.time()-start)
    assert np.all(np.isfinite(XX)) and summary['min_separation_m']>.54
    assert summary['max_quaternion_norm_error']<1e-4
    assert summary['settled_pointing_RMSE_deg']<5 and summary['centroid_RMSE_m']<.2
    name=f"{mode}_N{N}"+('_wind' if wind else '')
    np.savez_compressed(HERE/f'{name}.npz',metrics=Z,states=XX)
    np.savetxt(HERE/f'{name}_metrics.csv',Z,delimiter=',',header='time_s,centroid_error_m,pointing_error_deg,edge_rms_m,edge_max_m,min_separation_m,max_tilt_deg,accel_limited_fraction,motor_saturation_fraction,quat_norm_error,planarity_m,coherent_loss_dB,position_rms_m',comments='')
    print(json.dumps(summary),flush=True)
    return summary,Z,XX


def main():
    evidence=dict(execution='Independent Python implementation, not MATLAB or Simulink execution',analytic_checks=checks(),scenarios=[])
    print(json.dumps(evidence['analytic_checks']),flush=True)
    runs=[]
    for N,mode,wind in [(3,'geometric',False),(3,'paper',False),(6,'paper',False),(3,'paper',True)]:
        summary,Z,X=simulate(N,mode,wind); evidence['scenarios'].append(summary); runs.append((summary,Z,X))
        (HERE/'numerical_validation.json').write_text(json.dumps(evidence,indent=2))
    plot_existing(evidence)


def plot_existing(evidence):
    runs=[]
    for summary in evidence['scenarios']:
        name=f"{summary['mode']}_N{summary['N']}"+('_wind' if summary['wind'] else '')
        data=np.load(HERE/f'{name}.npz')
        runs.append((summary,data['metrics'],data['states']))
    fig,axes=plt.subplots(2,2,figsize=(12,7),layout='constrained')
    for s,Z,X in runs:
        label=f"{s['mode']}, N={s['N']}"+(', wind' if s['wind'] else '')
        for ax,col in zip(axes.flat,[1,2,3,5]): ax.plot(Z[:,0],Z[:,col],label=label,lw=1.3)
    for ax,title in zip(axes.flat,['Centroid error (m)','Pointing error (degrees)','Edge RMS error (m)','Minimum separation (m)']):
        ax.set(xlabel='Simulation time (s)',ylabel=title); ax.grid(alpha=.25)
    axes[0,0].legend(fontsize=8); axes[1,1].axhline(.54,color='r',ls='--',label='Body clearance threshold')
    fig.suptitle('Independent numerical reference — nonlinear quadrotors, sampled autopilots',fontsize=14)
    fig.savefig(HERE/'numerical_validation.png',dpi=160); plt.close(fig)
    s,Z,X=runs[1]; cfg=config(); fig=plt.figure(figsize=(10,8)); ax=fig.add_subplot(projection='3d')
    for i in range(3): ax.plot(X[:,0,i],X[:,1,i],X[:,2,i],label=f'Drone {i+1}',lw=1.)
    ax.scatter(*cfg['receiver'],marker='*',s=150,c='orange',edgecolors='black',label='Receiver')
    for k in np.linspace(0,len(Z)-1,7).astype(int):
        P=X[k,:3,:]; closed=P[:,[0,1,2,0]]; ax.plot(*closed,c='gray',alpha=.7,lw=.8)
        R,_,_,_=pose(P,X[k,3:6,:],cfg); pc=P.mean(axis=1); n=R[:,2]
        ax.quiver(*pc,*n,length=1.7,color='red',alpha=.6)
    ax.set(xlabel='x (m)',ylabel='y (m)',zlabel='z (m)',title='Paper-based mode: actual trajectories and formation normals\nIndependent Python reference, not Simulink output')
    ax.set_box_aspect([1,1,1.3]); ax.legend(); fig.savefig(HERE/'trajectory_preview.png',dpi=160); plt.close(fig)


if __name__=='__main__': main()
