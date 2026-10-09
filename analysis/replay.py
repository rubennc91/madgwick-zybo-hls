import numpy as np, sys
def make(T):
    f=lambda x: T(x)
    def rs(x): return T(1)/np.sqrt(x)
    def imu(q,gx,gy,gz,ax,ay,az,beta,dt):
        q0,q1,q2,q3=q
        qd1=f(.5)*(-q1*gx-q2*gy-q3*gz); qd2=f(.5)*(q0*gx+q2*gz-q3*gy)
        qd3=f(.5)*(q0*gy-q1*gz+q3*gx);  qd4=f(.5)*(q0*gz+q1*gy-q2*gx)
        if not (ax==0 and ay==0 and az==0):
            r=rs(ax*ax+ay*ay+az*az); ax*=r;ay*=r;az*=r
            _2q0=f(2)*q0;_2q1=f(2)*q1;_2q2=f(2)*q2;_2q3=f(2)*q3;_4q0=f(4)*q0;_4q1=f(4)*q1;_4q2=f(4)*q2;_8q1=f(8)*q1;_8q2=f(8)*q2
            q0q0=q0*q0;q1q1=q1*q1;q2q2=q2*q2;q3q3=q3*q3
            s0=_4q0*q2q2+_2q2*ax+_4q0*q1q1-_2q1*ay
            s1=_4q1*q3q3-_2q3*ax+f(4)*q0q0*q1-_2q0*ay-_4q1+_8q1*q1q1+_8q1*q2q2+_4q1*az
            s2=f(4)*q0q0*q2+_2q0*ax+_4q2*q3q3-_2q3*ay-_4q2+_8q2*q1q1+_8q2*q2q2+_4q2*az
            s3=f(4)*q1q1*q3-_2q1*ax+f(4)*q2q2*q3-_2q2*ay
            n2=s0*s0+s1*s1+s2*s2+s3*s3
            if n2>f(1e-20):
                r=rs(n2); s0*=r;s1*=r;s2*=r;s3*=r
            qd1-=beta*s0;qd2-=beta*s1;qd3-=beta*s2;qd4-=beta*s3
        q0+=qd1*dt;q1+=qd2*dt;q2+=qd3*dt;q3+=qd4*dt
        r=rs(q0*q0+q1*q1+q2*q2+q3*q3)
        return [q0*r,q1*r,q2*r,q3*r]
    def ahrs(q,gx,gy,gz,ax,ay,az,mx,my,mz,beta,dt):
        if mx==0 and my==0 and mz==0: return imu(q,gx,gy,gz,ax,ay,az,beta,dt)
        q0,q1,q2,q3=q
        qd1=f(.5)*(-q1*gx-q2*gy-q3*gz); qd2=f(.5)*(q0*gx+q2*gz-q3*gy)
        qd3=f(.5)*(q0*gy-q1*gz+q3*gx);  qd4=f(.5)*(q0*gz+q1*gy-q2*gx)
        if not (ax==0 and ay==0 and az==0):
            r=rs(ax*ax+ay*ay+az*az); ax*=r;ay*=r;az*=r
            r=rs(mx*mx+my*my+mz*mz); mx*=r;my*=r;mz*=r
            _2q0mx=f(2)*q0*mx;_2q0my=f(2)*q0*my;_2q0mz=f(2)*q0*mz;_2q1mx=f(2)*q1*mx
            _2q0=f(2)*q0;_2q1=f(2)*q1;_2q2=f(2)*q2;_2q3=f(2)*q3;_2q0q2=f(2)*q0*q2;_2q2q3=f(2)*q2*q3
            q0q0=q0*q0;q0q1=q0*q1;q0q2=q0*q2;q0q3=q0*q3;q1q1=q1*q1;q1q2=q1*q2;q1q3=q1*q3;q2q2=q2*q2;q2q3=q2*q3;q3q3=q3*q3
            hx=mx*q0q0-_2q0my*q3+_2q0mz*q2+mx*q1q1+_2q1*my*q2+_2q1*mz*q3-mx*q2q2-mx*q3q3
            hy=_2q0mx*q3+my*q0q0-_2q0mz*q1+_2q1mx*q2-my*q1q1+my*q2q2+_2q2*mz*q3-my*q3q3
            _2bx=np.sqrt(hx*hx+hy*hy)
            _2bz=-_2q0mx*q2+_2q0my*q1+mz*q0q0+_2q1mx*q3-mz*q1q1+_2q2*my*q3-mz*q2q2+mz*q3q3
            _4bx=f(2)*_2bx;_4bz=f(2)*_2bz
            h=f(.5)
            e1=f(2)*q1q3-_2q0q2-ax; e2=f(2)*q0q1+_2q2q3-ay
            m1=_2bx*(h-q2q2-q3q3)+_2bz*(q1q3-q0q2)-mx
            m2=_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-my
            m3=_2bx*(q0q2+q1q3)+_2bz*(h-q1q1-q2q2)-mz
            s0=-_2q2*e1+_2q1*e2-_2bz*q2*m1+(-_2bx*q3+_2bz*q1)*m2+_2bx*q2*m3
            s1=_2q3*e1+_2q0*e2-f(4)*q1*(f(1)-f(2)*q1q1-f(2)*q2q2-az)+_2bz*q3*m1+(_2bx*q2+_2bz*q0)*m2+(_2bx*q3-_4bz*q1)*m3
            s2=-_2q0*e1+_2q3*e2-f(4)*q2*(f(1)-f(2)*q1q1-f(2)*q2q2-az)+(-_4bx*q2-_2bz*q0)*m1+(_2bx*q1+_2bz*q3)*m2+(_2bx*q0-_4bz*q2)*m3
            s3=_2q1*e1+_2q2*e2+(-_4bx*q3+_2bz*q1)*m1+(-_2bx*q0+_2bz*q2)*m2+_2bx*q1*m3
            n2=s0*s0+s1*s1+s2*s2+s3*s3
            if n2>f(1e-20):
                r=rs(n2); s0*=r;s1*=r;s2*=r;s3*=r
            qd1-=beta*s0;qd2-=beta*s1;qd3-=beta*s2;qd4-=beta*s3
        q0+=qd1*dt;q1+=qd2*dt;q2+=qd3*dt;q3+=qd4*dt
        r=rs(q0*q0+q1*q1+q2*q2+q3*q3)
        return [q0*r,q1*r,q2*r,q3*r]
    return ahrs

def pl_inputs(raw,cfg,goff,moff,fs=1):
    """Reproduce nav_spi_ctrl out_sample + raw2float scaling (float32 scale), returns float64 array (n,9) of exact float32 values."""
    raw=np.asarray(raw,dtype=np.int64); n=len(raw)
    out=np.zeros((n,9),dtype=np.int64)
    for i in range(3):
        v=raw[:,i]-goff[i]; out[:,i]=-v if (cfg>>(3+i))&1 else v
        v=raw[:,3+i]; out[:,3+i]=-v if (cfg>>(9+i))&1 else v
    perm=(cfg>>6)&7
    P={1:(0,2,1),2:(1,0,2),3:(1,2,0),4:(2,0,1),5:(2,1,0)}.get(perm,(0,1,2))
    for i in range(3):
        v=raw[:,6+P[i]]-moff[P[i]]; out[:,6+i]=-v if (cfg>>i)&1 else v
    gs=np.float32(np.float32({0:8.75e-3,1:17.5e-3}.get(fs,70e-3))*np.float32(0.017453293))
    ams=np.float32(1.0/16384.0)
    f=out.astype(np.float32)
    f[:,:3]*=gs; f[:,3:]*=ams
    return f   # float32

def run(T,inp,beta_sw,dt=1/119.0,b0=1.0,b1=0.1,seed=(-1,0,0,0.2),cast_inputs=True):
    ahrs=make(T)
    q=[T(x) for x in seed]
    outq=[list(q)]
    for k in range(1,len(inp)):
        b=T(b0 if k<=beta_sw else b1)
        x=[T(v) for v in inp[k]]
        q=ahrs(q,*x,b,T(dt) if T is np.float64 else np.float32(dt))
        outq.append(list(q))
    return np.array(outq,dtype=np.float64)

def ang(a,b):
    a=a/np.linalg.norm(a,axis=1)[:,None];b=b/np.linalg.norm(b,axis=1)[:,None]
    r0=np.sum(a*b,axis=1)
    r1=a[:,0]*b[:,1]-a[:,1]*b[:,0]-a[:,2]*b[:,3]+a[:,3]*b[:,2]
    r2=a[:,0]*b[:,2]+a[:,1]*b[:,3]-a[:,2]*b[:,0]-a[:,3]*b[:,1]
    r3=a[:,0]*b[:,3]-a[:,1]*b[:,2]+a[:,2]*b[:,1]-a[:,3]*b[:,0]
    return 2*np.degrees(np.arctan2(np.sqrt(r1**2+r2**2+r3**2),np.abs(r0)))
