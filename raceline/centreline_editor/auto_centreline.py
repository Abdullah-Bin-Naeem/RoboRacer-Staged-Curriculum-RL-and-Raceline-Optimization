#!/usr/bin/env python3
"""Automatic reference centreline from a nav2 map.
   skeleton -> ridge projection (exact medial axis, no pixel staircase)
   -> curvature-adaptive smoothing (medial axis on straights, heavy smoothing at sharp corners)
   -> checks: straights flat, max |kappa| <= kappa_bound, inside corridor
   -> TUM reftrack CSV with ray-cast widths, seam on the longest straight."""
import sys, os, argparse, numpy as np, yaml
from PIL import Image
from scipy import ndimage
from scipy.signal import savgol_filter, find_peaks
from scipy.spatial import cKDTree
from skimage.morphology import skeletonize

def load(yaml_path):
    m=yaml.safe_load(open(yaml_path)); p=os.path.join(os.path.dirname(os.path.abspath(yaml_path)),m['image'])
    img=np.array(Image.open(p).convert('L')); 
    if m.get('negate',0): img=255-img
    return m,img

def skeleton_loop(opened,res,ox,oy):
    H,W=opened.shape; sk=skeletonize(opened,method='lee').astype(np.uint8); K=np.ones((3,3),int)
    for _ in range(800):
        nb=ndimage.convolve(sk.astype(int),K,mode='constant')-sk; ends=(sk==1)&(nb==1)
        if not ends.any(): break
        sk[ends]=0
    lab,n=ndimage.label(sk,structure=K)
    if n==0: sys.exit('no skeleton')
    sizes=ndimage.sum(sk,lab,range(1,n+1)); sk=lab==(np.argmax(sizes)+1)
    pts=np.argwhere(sk); tree=cKDTree(pts); vis=np.zeros(len(pts),bool); order=[0]; vis[0]=True; cur=0
    for _ in range(len(pts)-1):
        d,idx=tree.query(pts[cur],k=9); nxt=None
        for dd,ii in zip(d,idx):
            if not vis[ii] and dd<1.5: nxt=ii; break
        if nxt is None:
            d,idx=tree.query(pts[cur],k=40); c=[(dd,ii) for dd,ii in zip(d,idx) if not vis[ii]]
            if not c or c[0][0]>3: break
            nxt=c[0][1]
        vis[nxt]=True; order.append(nxt); cur=nxt
    P=pts[order]; return np.c_[ox+(P[:,1]+0.5)*res, oy+(H-P[:,0]-0.5)*res]

def resample(xy,ds):
    c=np.vstack([xy,xy[:1]]); d=np.hypot(*np.diff(c,axis=0).T); s=np.concatenate([[0],np.cumsum(d)]); L=s[-1]
    su=np.arange(0,L,ds); return np.c_[np.interp(su,s,c[:,0]),np.interp(su,s,c[:,1])],L
def sg(a,w):
    w=int(w)|1; n=len(a); e=np.vstack([a[-w:],a,a[:w]]); return savgol_filter(e,w,3,axis=0)[w:w+n]
def frame(P,ds):
    a=np.roll(P,1,0); b=np.roll(P,-1,0); xp=(b[:,0]-a[:,0])/(2*ds); yp=(b[:,1]-a[:,1])/(2*ds)
    xpp=(b[:,0]-2*P[:,0]+a[:,0])/ds**2; ypp=(b[:,1]-2*P[:,1]+a[:,1])/ds**2
    return (xp*ypp-yp*xpp)/np.power(xp**2+yp**2,1.5), np.arctan2(yp,xp)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('map_yaml'); ap.add_argument('-o','--out'); ap.add_argument('--ds',type=float,default=0.10)
    ap.add_argument('--open-px',type=int,default=21,help='opening kernel (px); removes wall notches narrower than this')
    ap.add_argument('--kappa-bound',type=float,default=1.5); ap.add_argument('--corner-window',type=float,default=3.5,help='m, smoothing at sharp corners')
    ap.add_argument('--free-min',type=int,default=250); ap.add_argument('--plot',action='store_true')
    a=ap.parse_args()
    m,img=load(a.map_yaml); res=float(m['resolution']); ox,oy=map(float,m['origin'][:2]); H,W=img.shape
    free=img>=a.free_min
    opened=ndimage.binary_opening(free,structure=np.ones((a.open_px,a.open_px),bool))
    D=ndimage.distance_transform_edt(opened)*res
    def Dat(x,y):
        c=(x-ox)/res-0.5; r=(H-1)-((y-oy)/res-0.5); return ndimage.map_coordinates(D,[[r],[c]],order=1,mode='nearest')[0]
    def isfree(x,y):
        i=int((x-ox)/res); j=int((y-oy)/res); r=H-1-j; return 0<=i<W and 0<=r<H and opened[r,i]
    def ray(x,y,nx,ny,maxd=3.0,st=0.01):
        d=0.0
        while d<maxd:
            d+=st
            if not isfree(x+nx*d,y+ny*d): return d-st
        return maxd
    ds=a.ds
    C,L=resample(skeleton_loop(opened,res,ox,oy),ds)
    # 1) medial ridge: transverse search for the max of the distance transform
    for it in range(4):
        k,psi=frame(C,ds); out=C.copy()
        for i,((x,y),p) in enumerate(zip(C,psi)):
            nx,ny=-np.sin(p),np.cos(p); ts=np.arange(-0.6,0.6001,0.01)
            vals=np.array([Dat(x+nx*t,y+ny*t) for t in ts]); t=ts[np.argmax(vals)]; out[i]=[x+nx*t,y+ny*t]
        C=sg(out,13); C,L=resample(C,ds)
    ridge=C.copy(); k_r,_=frame(ridge,ds)
    # 2) relax sharp corners outward until |kappa| <= kappa_target, never closer than min_clear to a wall
    kt=a.kappa_bound*0.85; min_clear=0.30; C=ridge.copy(); n=len(C); half=int(1.0/ds)
    g=np.exp(-0.5*(np.arange(-half,half+1)*ds/0.45)**2)
    for it in range(400):
        k,psi=frame(C,ds); over=np.abs(k)>kt
        if not over.any(): break
        push=np.zeros(n)
        for i in np.where(over)[0]:
            for j in range(-half,half+1): push[(i+j)%n]+=-np.sign(k[i])*g[j+half]
        push=np.clip(push,-1,1)*0.015
        nx,ny=-np.sin(psi),np.cos(psi); cand=C+np.c_[push*nx,push*ny]
        ok=np.array([Dat(x,y)>=min_clear for x,y in cand]); C[ok]=cand[ok]
        C=sg(C,9); C,L=resample(C,ds); n=len(C)
    print(f'corner relaxation: {it+1} iterations, target |kappa| <= {kt:.2f}')
    k,psi=frame(C,ds); n=len(C)
    # 3) checks
    clear=np.array([Dat(x,y) for x,y in C]); inside=np.array([isfree(x,y) for x,y in C])
    wl=np.array([ray(x,y,-np.sin(p),np.cos(p)) for (x,y),p in zip(C,psi)]); wr=np.array([ray(x,y,np.sin(p),-np.cos(p)) for (x,y),p in zip(C,psi)])
    kw,_=frame(sg(C,41),ds); small=np.abs(kw)<0.12; run=np.zeros(n,bool); i=0; best=(0,0)
    while i<n:
        if small[i]:
            j=i
            while j<n and small[j]: j+=1
            if (j-i)*ds>=2.5:
                run[i:j]=True
                if j-i>best[1]-best[0]: best=(i,j)
            i=j
        else: i+=1
    mid=(best[0]+best[1])//2; C,k,psi,wl,wr,run,clear,ridge=[np.roll(x,-mid,0) for x in (C,k,psi,wl,wr,run,clear,ridge)]
    S=np.arange(n)*ds
    print(f'lap {L:.2f} m, {n} pts, seam at ({C[0,0]:.2f},{C[0,1]:.2f}) on the longest straight')
    print(f'straights {run.sum()*ds:.1f} m: max|k| {np.abs(k[run]).max():.3f}  rms {np.sqrt(np.mean(k[run]**2)):.3f}   (targets <0.10 / <0.05)')
    print(f'lap max|k| {np.abs(k).max():.2f} (r {1/np.abs(k).max():.2f} m)  over {a.kappa_bound}: {(np.abs(k)>a.kappa_bound).sum()}  over 1.78: {(np.abs(k)>1.78).sum()}')
    print(f'inside corridor: {inside.all()}   min clearance to wall {clear.min():.2f} m   min corridor (wl+wr) {(wl+wr).min():.2f} m')
    pk,_=find_peaks(np.abs(k),height=0.35,distance=15)
    for p in pk: print(f'  corner s={S[p]:5.1f}  k={k[p]:+.2f}  r={1/abs(k[p]):.2f} m  clearance {clear[p]:.2f}  wl {wl[p]:.2f} wr {wr[p]:.2f}')
    out=a.out or os.path.join(os.path.dirname(os.path.abspath(a.map_yaml)),os.path.splitext(m['image'])[0]+'_centreline.csv')
    np.savetxt(out,np.c_[C,wr,wl,S,k],delimiter=',',fmt='%.4f',header='x_m,y_m,w_tr_right_m,w_tr_left_m,s_m,kappa_radpm',comments='# ')
    print('wrote',out)
    if a.plot:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig=plt.figure(figsize=(16,8.5)); ax=fig.add_subplot(2,1,1)
        ax.imshow(img[::-1,:].T,cmap='gray',extent=[oy,oy+H*res,ox,ox+W*res],origin='lower')
        ax.plot(ridge[:,1],ridge[:,0],color='#B4C5C1',lw=1.2,label='medial axis (ridge)')
        ax.plot(C[:,1],C[:,0],color='#B0175C',lw=2,label='reference centreline (adaptive)')
        bad=np.abs(k)>a.kappa_bound
        if bad.any(): ax.plot(C[bad,1],C[bad,0],'o',color='#B0175C',ms=5,label=f'|κ|>{a.kappa_bound}')
        ax.plot(C[0,1],C[0,0],'ko',ms=7,label='s=0'); ax.set_aspect('equal'); ax.legend(loc='lower right',fontsize=9)
        ax2=fig.add_subplot(2,1,2); ax2.plot(S,k,color='#B0175C',lw=1.4); ax2.fill_between(S,-3,3,where=run,color='#1F7A4D',alpha=.08)
        for v in (1.78,-1.78): ax2.axhline(v,ls='--',c='gray',lw=.9)
        for v in (a.kappa_bound,-a.kappa_bound): ax2.axhline(v,ls=':',c='gray',lw=.9)
        ax2.set_ylim(-3,3); ax2.set_ylabel('κ 1/m'); ax2.set_xlabel('s (m)')
        plt.tight_layout(); plt.savefig(os.path.splitext(out)[0]+'.png',dpi=100); print('wrote',os.path.splitext(out)[0]+'.png')
if __name__=='__main__': main()
